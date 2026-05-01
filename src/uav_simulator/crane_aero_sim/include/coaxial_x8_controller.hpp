#pragma once
/**
 * coaxial_x8_controller.hpp  —  Cascade position + SO(3) attitude controller
 *
 * Rewrites the reference cascadePID for X8 coaxial platform:
 *   - Outputs wrench (F_body_z, τ_body) — let the Allocator handle RPM conversion
 *   - Geometric SO(3) attitude control (Lee et al. 2010) — no Euler singularities
 *   - No k_F / k_T constants — the Allocator knows the aerodynamics
 *   - No δ₀ parameter — Mz consumes the 4-rotor-pair allocation DOF directly
 *
 * Control pipeline:
 *   (Pos_des, Vel_des, Acc_des, Yaw_des)   ← setpoints
 *        │
 *        ├─ Position PD → a_des_w (world-frame desired acceleration)
 *        │
 *        ├─ Thrust vector     : F_des_b = m · R_wb · (a_des_w + g·ẑ_w)
 *        │                      F_body_z = F_des_b · ẑ_b     (projection on body-z)
 *        │
 *        ├─ Desired attitude  : z_bdes = F_des_b / ||F_des_b||
 *        │                      yaw_ref from Yaw_des
 *        │                      R_des = [x_bdes, y_bdes, z_bdes]
 *        │
 *        └─ SO(3) attitude controller 
 *                 e_R = 0.5 · vee(R_des^T R - R^T R_des)
 *                 e_ω = ω_body - R^T R_des ω_des           (= ω_body if ω_des=0)
 *                 τ_body = -K_R · e_R - K_ω · e_ω + ω × (J ω)   (gyroscopic FF)
 *
 *   Output: (F_body_z, τ_body_xyz)  →  Allocator::allocate(...)
 *
 * Frame convention:
 *   world: z-up, right-handed
 *   body:  x-forward, y-left, z-up  (matches dynamics.hpp)
 *   R_wb = R_body2world, rotates a body-frame vector into world frame
 */
#include <ros/ros.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

struct RcInput {
    double pitch_stick    = 0.0;   // [-1,1]  +前 / -后
    double roll_stick     = 0.0;   // [-1,1]  +右 / -左
    double throttle_stick = 0.0;   // [-1,1]  +上 / -下  (中立=悬停)
    double yaw_stick      = 0.0;   // [-1,1]  +CW / -CCW (按你需要约定)
    bool   active         = false; // 是否处于 RC 模式
};

struct RcConfig {
    double max_vel_xy   = 3.0;                   // m/s
    double max_vel_z    = 2.0;                   // m/s
    double max_yaw_rate = 90.0 * M_PI / 180.0;   // rad/s
    double deadzone     = 0.05;                  // 摇杆死区
    double expo         = 0.3;                   // 0=线性, 0.3~0.5 推荐
};

class CoaxialX8Controller
{
public:
    struct Gains
    {
        // Position (outer) loop — second-order response design
        double pos_stable_time  = 3.0;   // s
        double pos_damping      = 0.90;
        double alt_stable_time  = 2.0;   // z-axis can be tighter
        double alt_damping      = 0.80;
        double ki_z             = 0.0;   // altitude integral (set >0 if hover droop)

        // Attitude (inner) loop — SO(3) gains on rotation error & angular rate
        double att_stable_time  = 0.5;   // s — much faster than outer loop
        double att_damping      = 0.9;
        double yaw_rel_kp       = 1.0;   // yaw channel scale (relative to roll/pitch)
        double yaw_rel_kd       = 1.0;
    };

    struct Limits
    {
        double max_pos_err     = 10.0;   // m
        double max_vel_err     = 10.0;   // m/s
        double max_acc_xy      = 10.0;   // m/s^2
        double max_acc_z_up    =  4.0;   // × g
        double max_acc_z_down  = -0.8;   // × g
        double max_tilt_rad    = 0.525;  // ~30 deg
        double z_integral_lim  = 2.0;

        double max_thrust_N    = 0.0;    // 0 → auto from mass (4·m·g)
        double max_torque_xy_Nm = 20.0;
        double max_torque_z_Nm  = 20.0;  // now big: yaw uses full Q-allocation DOF,
                                         // no longer tiny like in the reference code
        double max_yaw_rate_rad = 30.0 * M_PI / 180.0;   // 新增：先限制到 30 deg/s
    };

    struct Params
    {
        double mass = 1.9;               // kg
        Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
        double g = 9.81;
    };

    struct Output
    {
        double          thrust_body_z;   // N — total demanded thrust along body +z
        Eigen::Vector3d torque_body;     // N·m — body-frame torque demand [Mx, My, Mz]
    };

private:
    double dt_;
    Params p_;
    Gains  g_;
    Limits lim_;

    // Derived gains
    Eigen::Vector3d Kp_att_, Kd_att_;
    double Kp_pos_xy_, Kd_pos_xy_;
    double Kp_pos_z_,  Kd_pos_z_;

    // Setpoints
    Eigen::Vector3d pos_des_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_des_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_des_ = Eigen::Vector3d::Zero();
    double          yaw_des_ = 0.0;

    // Feedback
    Eigen::Vector3d    pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d    vel_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_   = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d    R_   = Eigen::Matrix3d::Identity();
    Eigen::Vector3d    omega_b_ = Eigen::Vector3d::Zero();

    // State
    double z_integral_ = 0.0;

    double yaw_cmd_ = 0.0;
    bool yaw_initialized_ = false;

    RcConfig rc_cfg_;
    bool xy_hold_latched_ = false;
    bool z_hold_latched_  = false;
    bool rc_yaw_initialized_ = false;

    Eigen::Vector3d vel_des_prev_ = Eigen::Vector3d::Zero();
    bool vel_des_initialized_ = false;

    // Output cache
    Output out_{};

public:
    explicit CoaxialX8Controller(double control_rate_hz = 200.0)
        : dt_(1.0 / control_rate_hz)
    {
        recomputeGains();
    }

    // ================================================================
    // Configuration
    // ================================================================
    void setRate(double hz)              { dt_ = 1.0 / hz; }
    void setParams(const Params& p)      { p_ = p; recomputeGains(); }
    void setGains(const Gains& g)        { g_ = g; recomputeGains(); }
    void setLimits(const Limits& l)      { lim_ = l; }
    void setRcConfig(const RcConfig& c)  { rc_cfg_ = c; }

    // ================================================================
    // Inputs
    // ================================================================
    void setFeedback(const Eigen::Vector3d& pos_w,
                     const Eigen::Vector3d& vel_w,
                     const Eigen::Quaterniond& q_wb,
                     const Eigen::Vector3d& omega_body)
    {
        pos_     = pos_w;
        vel_     = vel_w;
        q_       = q_wb;
        q_.normalize();
        R_       = q_.toRotationMatrix();       // body → world
        omega_b_ = omega_body;
    }

    void setSetpoint(const Eigen::Vector3d& pos_des,
                     const Eigen::Vector3d& vel_des,
                     const Eigen::Vector3d& acc_des,
                     double yaw_des)
    {
        pos_des_ = pos_des;
        vel_des_ = vel_des;
        acc_des_ = acc_des;
        // yaw_des_ = yaw_des;
        yaw_des_ = wrapPi(yaw_des);
    }

    void updateFromRC(const RcInput& rc)
    {
        // ---- 死区 + expo 整形 ----
        auto shape = [&](double s) {
            if (std::abs(s) < rc_cfg_.deadzone) return 0.0;
            double sign = (s > 0) ? 1.0 : -1.0;
            double mag  = (std::abs(s) - rc_cfg_.deadzone) /
                        (1.0 - rc_cfg_.deadzone);
            return sign * ((1.0 - rc_cfg_.expo) * mag
                        +  rc_cfg_.expo * mag * mag * mag);
        };

        const double p = shape(rc.pitch_stick);
        const double r = shape(rc.roll_stick);
        const double t = shape(rc.throttle_stick);
        const double y = shape(rc.yaw_stick);

        // ============================================================
        // 安全因子 1: TWR 检查 — 推重比小时全局收紧
        // ============================================================
        const double F_max = (lim_.max_thrust_N > 0.0)
                        ? lim_.max_thrust_N
                        : 1.2 * p_.mass * p_.g;
        const double TWR = F_max / (p_.mass * p_.g);
        const double safety = std::clamp((TWR - 1.0) / 0.6, 0.01, 1.0);

        // ============================================================
        // 安全因子 2: 当前 tilt 余量 — 接近 max_tilt 时压制水平指令
        // ============================================================
        const double z_b_z     = R_(2, 2);
        const double tilt_curr = std::acos(std::clamp(z_b_z, -1.0, 1.0));
        const double tilt_margin = std::clamp(
            (lim_.max_tilt_rad - tilt_curr) / (0.3 * lim_.max_tilt_rad),
            0.0, 1.0);

        // ============================================================
        // 安全因子 3: 角速度反向衰减 — 旋转太快时降低 yaw 指令
        // ============================================================
        const double omega_z_lim = 1.0;
        const double yaw_overshoot_margin = std::clamp(
            1.0 - (std::abs(omega_b_.z()) - 0.5) / (omega_z_lim - 0.5),
            0.0, 1.0);

        // ============================================================
        // 安全因子 4 (新增): body 角速度反向衰减 —
        // 当 roll/pitch 角速度还没收敛时,降低水平速度指令幅值
        // 配合下面的 slew rate,防止"飞机还在朝一边转,你又指令它朝另一边"
        // ============================================================
        const double omega_xy_norm = std::hypot(omega_b_.x(), omega_b_.y());
        const double omega_xy_lim  = 0.8;   // rad/s,~46 deg/s 触发衰减
        const double omega_xy_margin = std::clamp(
            1.0 - (omega_xy_norm - 0.4) / (omega_xy_lim - 0.4),
            0.2, 1.0);   // 不衰到 0,留点机动权

        // 综合系数
        const double xy_scale  = safety * tilt_margin * omega_xy_margin;
        const double yaw_scale = safety * yaw_overshoot_margin;

        // ---- 当前 yaw（用于初始化 + 旋转 body→world） ----
        const double yaw_curr = std::atan2(R_(1, 0), R_(0, 0));
        if (!rc_yaw_initialized_) {
            yaw_cmd_ = yaw_curr;
            yaw_des_ = yaw_curr;
            yaw_initialized_    = true;
            rc_yaw_initialized_ = true;
        }

        // ---- 偏航：杆动 = 积分 yaw rate;松杆 = 跟随实际朝向直到角速度小 ----
        const bool yaw_active = std::abs(y) > 1e-6;
        if (yaw_active) {
            const double yaw_rate_cmd = y *
                std::min(rc_cfg_.max_yaw_rate, lim_.max_yaw_rate_rad) *
                yaw_scale;
            yaw_cmd_ = wrapPi(yaw_cmd_ + yaw_rate_cmd * dt_);
            yaw_des_ = yaw_cmd_;
        } else {
            if (std::abs(omega_b_.z()) > 0.05) {
                yaw_cmd_ = yaw_curr;
                yaw_des_ = yaw_curr;
            }
        }

        // ---- 摇杆 → body 系期望速度 (带安全缩放) ----
        const Eigen::Vector3d vel_body(
            p * rc_cfg_.max_vel_xy * xy_scale,
            -r * rc_cfg_.max_vel_xy * xy_scale,
            t * rc_cfg_.max_vel_z  * safety);

        // body → world,只用 yaw 分量
        const double cy = std::cos(yaw_curr);
        const double sy = std::sin(yaw_curr);
        Eigen::Vector3d vel_des_target(
            cy * vel_body.x() - sy * vel_body.y(),
            sy * vel_body.x() + cy * vel_body.y(),
            vel_body.z());

        // ============================================================
        // 关键修复: SETPOINT SLEW RATE LIMITER
        // 不允许 vel_des_ 在一帧内做大跳变
        // 让命令的变化率和姿态环可达带宽匹配
        // ============================================================

        // 首次进入 RC: 把 prev 对齐到当前实际速度,避免冷启动冲击
        if (!vel_des_initialized_) {
            vel_des_prev_        = vel_;
            vel_des_initialized_ = true;
        }

        // ---- 水平 slew rate: 比 max_acc_xy 再保守 30%,留余量给姿态环 ----
        const double vel_slew_xy = 0.4 * lim_.max_acc_xy;
        const double dv_max_xy   = vel_slew_xy * dt_;

        Eigen::Vector2d dv_xy(vel_des_target.x() - vel_des_prev_.x(),
                            vel_des_target.y() - vel_des_prev_.y());
        const double dv_xy_norm = dv_xy.norm();
        if (dv_xy_norm > dv_max_xy && dv_xy_norm > 1e-9) {
            dv_xy *= dv_max_xy / dv_xy_norm;
        }
        vel_des_.x() = vel_des_prev_.x() + dv_xy.x();
        vel_des_.y() = vel_des_prev_.y() + dv_xy.y();

        // ---- 垂直 slew rate ----
        const double vel_slew_z = 0.5 * std::min(lim_.max_acc_z_up,
                                                std::abs(lim_.max_acc_z_down));
        const double dv_max_z   = vel_slew_z * dt_;
        const double dvz        = vel_des_target.z() - vel_des_prev_.z();
        vel_des_.z() = vel_des_prev_.z() + std::clamp(dvz, -dv_max_z, dv_max_z);

        // ---- acc_des 设为 setpoint 真实导数,作为外环前馈 ----
        // 这样反打时姿态环能立刻知道目标方向变了,而不是等 vel error 才反应
        acc_des_ = (vel_des_ - vel_des_prev_) / std::max(dt_, 1e-4);

        vel_des_prev_ = vel_des_;

        // ============================================================
        // 位置 latch 保持原逻辑
        // ============================================================

        // ---- 位置 latch (xy) ----
        const bool xy_active = (std::abs(p) > 1e-6) || (std::abs(r) > 1e-6);
        if (xy_active) {
            pos_des_.x() = pos_.x();   // 打杆时 P 项恒为 0,只剩 D 跟踪速度
            pos_des_.y() = pos_.y();
            xy_hold_latched_ = false;
        } else {
            if (!xy_hold_latched_) {
                pos_des_.x() = pos_.x();
                pos_des_.y() = pos_.y();
                if (std::hypot(vel_.x(), vel_.y()) < 0.2) {
                    xy_hold_latched_ = true;
                }
            }
        }

        // ---- 位置 latch (z) ----
        const bool z_active = std::abs(t) > 1e-6;
        if (z_active) {
            pos_des_.z() = pos_.z();
            z_integral_  = 0.0;
            z_hold_latched_ = false;
        } else if (!z_hold_latched_) {
            pos_des_.z() = pos_.z();
            if (std::abs(vel_.z()) < 0.1) {
                z_hold_latched_ = true;
            }
        }

        // ---- (可选) 调试日志 ----
        ROS_INFO_THROTTLE(0.5,
            "RC safety: TWR=%.2f safety=%.2f tilt_margin=%.2f "
            "omega_xy_margin=%.2f yaw_margin=%.2f",
            TWR, safety, tilt_margin, omega_xy_margin, yaw_overshoot_margin);
    }

    // 切出 RC 模式时调一下,防止下次进入还沿用旧的 latch 状态
    void resetRcLatch()
    {
        xy_hold_latched_      = false;
        z_hold_latched_       = false;
        rc_yaw_initialized_   = false;
    }

    Output run()
    {
        // ------------- OUTER LOOP: position / velocity → desired accel -------
        Eigen::Vector3d e_p = clamp3(pos_des_ - pos_, lim_.max_pos_err);
        Eigen::Vector3d e_v = clamp3(vel_des_ - vel_, lim_.max_vel_err);

        // Altitude integral (anti-windup)
        z_integral_ = std::clamp(z_integral_ + e_p.z() * dt_,
                                -lim_.z_integral_lim, lim_.z_integral_lim);

        double a_z = Kp_pos_z_ * e_p.z()
                + Kd_pos_z_ * e_v.z()
                + g_.ki_z   * z_integral_
                + acc_des_.z();
        // a_z = std::clamp(a_z, lim_.max_acc_z_down * p_.g, lim_.max_acc_z_up * p_.g);
        a_z = std::clamp(a_z, lim_.max_acc_z_down, lim_.max_acc_z_up);

        // double a_x = std::clamp(Kp_pos_xy_ * e_p.x() + Kd_pos_xy_ * e_v.x() + acc_des_.x(),
        //                         -lim_.max_acc_xy, lim_.max_acc_xy);
        // double a_y = std::clamp(Kp_pos_xy_ * e_p.y() + Kd_pos_xy_ * e_v.y() + acc_des_.y(),
        //                         -lim_.max_acc_xy, lim_.max_acc_xy);

        double a_x_raw = Kp_pos_xy_ * e_p.x() + Kd_pos_xy_ * e_v.x() + acc_des_.x();
        double a_y_raw = Kp_pos_xy_ * e_p.y() + Kd_pos_xy_ * e_v.y() + acc_des_.y();

        Eigen::Vector2d a_xy(a_x_raw, a_y_raw);

        // 根据推力余量动态限制水平加速度
        const double F_max_pre = (lim_.max_thrust_N > 0.0)
                            ? lim_.max_thrust_N
                            : 1.2 * p_.mass * p_.g;

        // 留 10% 推力余量给姿态分配和电机动态
        // const double F_use = 0.90 * F_max_pre;

        // const double az_total = std::max(1.0, p_.g + a_z);

        // auto sqr = [](double v) { return v * v; };

        // double axy_by_thrust = 0.0;
        // const double temp = sqr(F_use / p_.mass) - sqr(az_total);
        // if (temp > 0.0) {
        //     axy_by_thrust = std::sqrt(temp);
        // }

        // const double axy_by_tilt = az_total * std::tan(lim_.max_tilt_rad);

        // const double axy_lim = std::min({
        //     lim_.max_acc_xy,
        //     axy_by_thrust,
        //     axy_by_tilt
        // });

        // if (a_xy.norm() > axy_lim && a_xy.norm() > 1e-6) {
        //     a_xy *= axy_lim / a_xy.norm();
        // }

        // 修正前: F_use = 0.90 × F_max
        // 这是错的 — 重力补偿之后才轮到水平 + 姿态修正
        // 当前 tilt 下重力补偿真实需求
        // const double cos_curr_clamp = std::max(0.5, R_(2, 2));   // 防 1/0
        // const double F_grav_need = p_.mass * p_.g / cos_curr_clamp;

        // // 留 20% 余量给姿态扭矩(扭矩 = 推力差 × 力臂,会吃掉推力)
        // const double F_att_reserve = 0.20 * F_max_pre;

        // // 真正能给水平加速度用的推力
        // const double F_horiz_avail = std::max(0.0,
        //     F_max_pre - F_grav_need - F_att_reserve);

        // const double az_total = std::max(1.0, p_.g + a_z);
        // const double axy_by_thrust = F_horiz_avail / p_.mass;   // 直接用,别再算 sqrt
        // const double axy_by_tilt   = az_total * std::tan(lim_.max_tilt_rad);
        // const double axy_lim = std::min({lim_.max_acc_xy, axy_by_thrust, axy_by_tilt});

        // if (a_xy.norm() > axy_lim && a_xy.norm() > 1e-6) {
        //     a_xy *= axy_lim / a_xy.norm();
        // }
        
        // double a_x = a_xy.x();
        // double a_y = a_xy.y();


        // 给姿态扭矩留 15% 余量(更小一点,你 TWR 本来就紧)
        double a_x, a_y;
        const double F_att_reserve = 0.15 * F_max_pre;
        const double F_avail       = F_max_pre - F_att_reserve;

        // 必要的检查: 余量必须够撑住 hover
        if (F_avail <= p_.mass * p_.g) {
            // 连水平 hover 都撑不住 — TWR 太低,这个平台没法飞
            // 只允许垂直,水平加速度强制为 0
            a_x = 0.0;
            a_y = 0.0;
        } else {
            // 推力约束下的最大倾角
            const double cos_tilt_min  = (p_.mass * p_.g) / F_avail;
            const double tilt_max_thr  = std::acos(std::clamp(cos_tilt_min, 0.0, 1.0));

            // 综合限制: tilt 上限 = min(几何限制, 推力限制)
            const double tilt_eff = std::min(lim_.max_tilt_rad, tilt_max_thr);

            // 由 tilt 限制得出 axy 上限
            const double axy_by_tilt   = p_.g * std::tan(tilt_eff);
            const double axy_lim       = std::min(lim_.max_acc_xy, axy_by_tilt);

            Eigen::Vector2d a_xy(a_x_raw, a_y_raw);
            if (a_xy.norm() > axy_lim && a_xy.norm() > 1e-6) {
                a_xy *= axy_lim / a_xy.norm();
            }
            a_x = a_xy.x();
            a_y = a_xy.y();
        }

        // ------------- THRUST VECTOR in world frame -------------------------
        // F_des_w = m * (a_des_w + g·z_w)
        Eigen::Vector3d F_des_w(p_.mass * a_x,
                                p_.mass * a_y,
                                p_.mass * (a_z + p_.g));

        // Guard against zero-thrust edge case
        const double F_norm = F_des_w.norm();
        if (F_norm < 1e-6) {
            out_.thrust_body_z = 0.0;
            out_.torque_body.setZero();
            return out_;
        }

        // ------------- DESIRED ATTITUDE R_des --------------------------------
        // Body-z axis in world frame = direction of desired thrust vector
        Eigen::Vector3d z_b_des = F_des_w.normalized();

        // Enforce tilt limit by pulling z_b_des toward world-z
        const Eigen::Vector3d z_w(0, 0, 1);
        const double cos_tilt = std::clamp(z_b_des.dot(z_w), -1.0, 1.0);
        const double tilt     = std::acos(cos_tilt);
        if (tilt > lim_.max_tilt_rad) {
            Eigen::Vector3d axis = z_w.cross(z_b_des);
            if (axis.norm() > 1e-9) {
                axis.normalize();
                Eigen::AngleAxisd aa(lim_.max_tilt_rad, axis);
                z_b_des = aa * z_w;
            }
        }

        if (!yaw_initialized_) {
            yaw_cmd_ = yaw_des_;
            yaw_initialized_ = true;
        }

        double yaw_err_cmd = wrapPi(yaw_des_ - yaw_cmd_);
        double max_step = lim_.max_yaw_rate_rad * dt_;
        yaw_cmd_ = wrapPi(yaw_cmd_ + std::clamp(yaw_err_cmd, -max_step, max_step));

        // yaw-reference body-x: project world yaw direction into the plane ⊥ z_b_des
        Eigen::Vector3d x_c(std::cos(yaw_cmd_), std::sin(yaw_cmd_), 0.0);
        Eigen::Vector3d y_b_des = z_b_des.cross(x_c);
        if (y_b_des.norm() < 1e-6) {
            y_b_des = z_b_des.cross(Eigen::Vector3d(0, 1, 0));
        }
        y_b_des.normalize();
        Eigen::Vector3d x_b_des = y_b_des.cross(z_b_des);

        Eigen::Matrix3d R_des;
        R_des.col(0) = x_b_des;
        R_des.col(1) = y_b_des;
        R_des.col(2) = z_b_des;

        // ============================================================
        // SAFETY LAYER: Tilt-prioritized SO(3) error
        // 把姿态误差分解成 tilt 部分(关乎飞行安全) + yaw 部分(只影响朝向)
        // tilt 大时降低 yaw 权重,把所有力矩余量留给扶正
        // ============================================================
        const Eigen::Vector3d z_b      = R_.col(2);
        const Eigen::Vector3d z_b_des_ = R_des.col(2);

        // Tilt error: 把 z_b 旋到 z_b_des 所需的旋转向量(在 body 系)
        Eigen::Vector3d e_R_tilt = Eigen::Vector3d::Zero();
        const double cos_tilt_err = std::clamp(z_b.dot(z_b_des_), -1.0, 1.0);
        const double tilt_err     = std::acos(cos_tilt_err);
        if (tilt_err > 1e-6) {
            Eigen::Vector3d axis_w = z_b.cross(z_b_des_);   // world frame
            if (axis_w.norm() > 1e-9) {
                axis_w.normalize();
                e_R_tilt = R_.transpose() * (tilt_err * axis_w);   // → body frame
            }
        }

        // 完整 SO(3) 误差 (Lee 2010)
        // ============================================================
        // R_des slew limiter — 期望姿态相对当前姿态的角距离不能太大
        // 也不能在帧间做大跳变
        // ============================================================
        {
            Eigen::Matrix3d R_err = R_.transpose() * R_des;
            Eigen::AngleAxisd aa_err(R_err);
            double ang = aa_err.angle();
            Eigen::Vector3d axis = aa_err.axis();

            // 关键参数: 期望姿态最多领先当前姿态 0.25 rad (~14°)
            // 物理上界: 你的 α_max = max_torque/J = 500/500 = 1 rad/s²
            // 0.25 rad 对应"控制律一定能在 1 秒内消掉"
            const double max_att_lead = 0.1;

            if (ang > max_att_lead && std::isfinite(ang)) {
                Eigen::AngleAxisd aa_clamped(max_att_lead, axis);
                R_des = R_ * aa_clamped.toRotationMatrix();
            }
        }
        Eigen::Matrix3d skew_eR =
            0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
        Eigen::Vector3d e_R_full(skew_eR(2, 1), skew_eR(0, 2), skew_eR(1, 0));

        // Yaw 部分 = full - tilt
        Eigen::Vector3d e_R_yaw = e_R_full - e_R_tilt;

        // tilt > tilt_warn 时线性降低 yaw 权重,到 max_tilt 时归零
        const double tilt_warn = 0.5 * lim_.max_tilt_rad;
        double yaw_weight = 1.0;
        if (tilt_err > tilt_warn) {
            yaw_weight = std::clamp(
                1.0 - (tilt_err - tilt_warn) / (lim_.max_tilt_rad - tilt_warn),
                0.0, 1.0);
        }

        Eigen::Vector3d e_R = e_R_tilt + yaw_weight * e_R_yaw;

        // angular-velocity error: ω_des = 0 (no feedforward yet)
        Eigen::Vector3d e_omega = omega_b_;

        // ------------- TORQUE: τ = -Kp · e_R - Kd · e_ω + ω × (J · ω) --------
        Eigen::Vector3d tau_body;
        tau_body(0) = -Kp_att_(0) * e_R(0) - Kd_att_(0) * e_omega(0);
        tau_body(1) = -Kp_att_(1) * e_R(1) - Kd_att_(1) * e_omega(1);
        tau_body(2) = -Kp_att_(2) * e_R(2) - Kd_att_(2) * e_omega(2);

        // Gyroscopic feedforward — small for axisymmetric multirotor but cheap
        tau_body += omega_b_.cross(p_.inertia * omega_b_);

        tau_body(0) = std::clamp(tau_body(0), -lim_.max_torque_xy_Nm, lim_.max_torque_xy_Nm);
        tau_body(1) = std::clamp(tau_body(1), -lim_.max_torque_xy_Nm, lim_.max_torque_xy_Nm);
        tau_body(2) = std::clamp(tau_body(2), -lim_.max_torque_z_Nm,  lim_.max_torque_z_Nm);

        // ------------- THRUST along actual body-z ----------------------------
        // Project F_des_w onto the *current* body-z axis (standard geometric ctrl)
        // const Eigen::Vector3d z_b_curr = R_.col(2);
        // double F_body_z = F_des_w.dot(z_b_curr);

        // const double F_max = (lim_.max_thrust_N > 0.0)
        //                 ? lim_.max_thrust_N
        //                 : 1.2 * p_.mass * p_.g;

        // F_body_z = std::clamp(F_body_z, 0.0, F_max);

        const Eigen::Vector3d z_b_curr = R_.col(2);

        const double F_max = (lim_.max_thrust_N > 0.0)
                        ? lim_.max_thrust_N
                        : 1.2 * p_.mass * p_.g;

        // 翻滚保护: cos_curr 太小说明已经在翻,这时候疯狂加推力没用
        // 反而把推力让给姿态扭矩,先扶正再说
        const double cos_curr_real = z_b_curr.z();
        double F_body_z;

        if (cos_curr_real < 0.5) {
            // tilt > 60°,已经在翻 —— 推力收到 hover 量级,让分配器的扭矩占优
            F_body_z = std::min(p_.mass * p_.g, F_max);
        } else {
            // 正常状态: 标准 geometric control 投影
            F_body_z = F_des_w.dot(z_b_curr);
            F_body_z = std::clamp(F_body_z, 0.0, F_max);
        }
        
        // ------------- OUTPUT -----------------------------------------------
        out_.thrust_body_z = F_body_z;
        out_.torque_body   = tau_body;

        // ================================================================
        // DEBUG LOG (throttled to ~10 Hz at 200 Hz control)
        // ================================================================
        {
            const double e_p_norm = (pos_des_ - pos_).norm();
            const double e_v_norm = (vel_des_ - vel_).norm();
            const double e_R_norm = e_R.norm();
            const double e_w_norm = e_omega.norm();

            const double tilt_curr = std::acos(std::clamp(z_b_curr.z(), -1.0, 1.0));
            const double tilt_cmd  = std::acos(std::clamp(z_b_des.z(),  -1.0, 1.0));

            const double yaw_curr = std::atan2(R_(1,0), R_(0,0));

            const bool sat_Fz   = (F_body_z >= F_max - 1e-3) || (F_body_z <= 1e-3);
            const bool sat_txy  = (std::abs(tau_body(0)) >= lim_.max_torque_xy_Nm - 1e-3) ||
                                (std::abs(tau_body(1)) >= lim_.max_torque_xy_Nm - 1e-3);
            const bool sat_tz   = std::abs(tau_body(2)) >= lim_.max_torque_z_Nm - 1e-3;
            const bool sat_tilt = tilt_cmd >= lim_.max_tilt_rad - 1e-4;

            // ROS_INFO_THROTTLE(0.1,
            //     "\n"
            //     "  pos err|vel err : %6.3f | %6.3f m\n"
            //     "  F_des_w norm    : %8.2f N    (hover = %6.1f N)\n"
            //     "  F_body_z        : %8.2f N  %s\n"
            //     "  tau_body xyz    : %7.2f %7.2f %7.2f N m  %s %s\n"
            //     "  tilt cur|cmd|max: %5.2f | %5.2f | %5.2f deg  %s\n"
            //     "  e_R | e_omega   : %6.3f rad | %6.3f rad/s\n"
            //     "  yaw cur|cmd|des : %6.2f | %6.2f | %6.2f deg\n"
            //     "  yaw_weight      : %.2f\n"
            //     "  z_integral      : %6.3f",
            //     e_p_norm, e_v_norm,
            //     F_des_w.norm(), p_.mass * p_.g,
            //     F_body_z, sat_Fz ? "<SAT>" : "",
            //     tau_body(0), tau_body(1), tau_body(2),
            //     sat_txy ? "<SAT_XY>" : "", sat_tz ? "<SAT_Z>" : "",
            //     tilt_curr * 180.0 / M_PI,
            //     tilt_cmd  * 180.0 / M_PI,
            //     lim_.max_tilt_rad * 180.0 / M_PI,
            //     sat_tilt ? "<SAT>" : "",
            //     e_R_norm, e_w_norm,
            //     yaw_curr * 180.0 / M_PI,
            //     yaw_cmd_ * 180.0 / M_PI,
            //     yaw_des_ * 180.0 / M_PI,
            //     yaw_weight,
            //     z_integral_);
        }

        return out_;
    }
    
    // ================================================================
    // Accessors
    // ================================================================
    const Output& lastOutput() const { return out_; }
    double        z_integral()  const { return z_integral_; }
    void          resetIntegrals() { z_integral_ = 0.0; }

private:
    void recomputeGains()
    {
        // Second-order spec: wn = 4.6 / (ζ · t_s)
        const double wn_xy  = 4.6 / (g_.pos_damping * g_.pos_stable_time);
        Kp_pos_xy_ = wn_xy * wn_xy;
        Kd_pos_xy_ = 2.0 * g_.pos_damping * wn_xy;

        const double wn_z   = 4.6 / (g_.alt_damping * g_.alt_stable_time);
        Kp_pos_z_  = wn_z * wn_z;
        Kd_pos_z_  = 2.0 * g_.alt_damping * wn_z;

        const double wn_a   = 4.6 / (g_.att_damping * g_.att_stable_time);
        const double kp_a   = wn_a * wn_a;
        const double kd_a   = 2.0 * g_.att_damping * wn_a;

        // Attitude gains on rotation vector (rad) → scaled by inertia inside τ
        // Rows of J multiplied in explicitly so users can tune by time constant,
        // not by raw torque coefficients.
        Kp_att_ << p_.inertia(0,0) * kp_a,
                   p_.inertia(1,1) * kp_a,
                   p_.inertia(2,2) * kp_a * g_.yaw_rel_kp;
        Kd_att_ << p_.inertia(0,0) * kd_a,
                   p_.inertia(1,1) * kd_a,
                   p_.inertia(2,2) * kd_a * g_.yaw_rel_kd;
    }

    static Eigen::Vector3d clamp3(const Eigen::Vector3d& v, double lim)
    {
        return Eigen::Vector3d(std::clamp(v.x(), -lim, lim),
                               std::clamp(v.y(), -lim, lim),
                               std::clamp(v.z(), -lim, lim));
    }

    static double wrapPi(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

};