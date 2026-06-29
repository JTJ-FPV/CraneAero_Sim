// #pragma once
// /**
//  * coaxial_x8_dynamics.hpp  —  X8 coaxial multirotor dynamics
//  */
// #include <array>
// #include <algorithm>
// #include <cmath>
// #include "dynamics.hpp"
// #include "coaxial_rotor_model.hpp"
// #include "fuselage_aero_model.hpp"

// class coaxial_x8_dynamics : public dynamics
// {
// public:
//     static constexpr int N_ROTORS = 8;
//     static constexpr int N_PAIRS  = 4;

//     // Pair structure: pairs_[p] = {upper_rotor_idx, lower_rotor_idx}
//     static constexpr std::array<std::pair<int,int>, N_PAIRS> PAIRS = {
//         {{0,1}, {2,3}, {4,5}, {6,7}}
//     };

//     // Yaw sign per pair: +1 if pair's CFD Q convention matches airframe yaw,
//     // -1 if opposite spin direction.
//     // With spin_ = {+1,-1, -1,+1, +1,-1, -1,+1}, pairs 0,2 match CFD, pairs 1,3 are flipped.
//     static constexpr std::array<double, N_PAIRS> PAIR_YAW_SIGN = {+1.0, -1.0, +1.0, -1.0};

// private:
//     CoaxialRotorModel aero_model_;   // CFD-based RBF (built at construction)
//     FuselageAeroModel fuselage_aero_;      //  CFD-based RBF fuselage
    
//     Eigen::Matrix<double, N_ROTORS, 1> motor_rpm_cmd_ = Eigen::Matrix<double, N_ROTORS, 1>::Zero();
//     Eigen::Matrix<double, N_ROTORS, 1> motor_rpm_     = Eigen::Matrix<double, N_ROTORS, 1>::Zero();

//     Eigen::Matrix<double, N_ROTORS, 3> motor_pos_;
//     Eigen::Matrix<double, N_ROTORS, 3> motor_axis_;

//     double arm_length_    = 1.65;
//     double z_sep_         = 0.40;    // vertical separation between upper and lower rotor
//     double min_rpm_       = CoaxialRotorModel::rpmMin();  // [BUG-2] match CFD range
//     double max_rpm_       = CoaxialRotorModel::rpmMax();
//     double motor_tau_     = 0.03;    // first-order motor time constant [s]
//     double g_             = 9.81;

//     Eigen::Vector3d drag_linear_body_  = Eigen::Vector3d(0.08, 0.08, 0.12);
//     Eigen::Vector3d drag_angular_body_ = Eigen::Vector3d(0.0008, 0.0008, 0.0012);

// public:
//     using dynamics::dynamics;

//     void init(const Eigen::Vector3d& p_w, const Eigen::Vector4d& quat_wxyz)
//     {
//         initialize(p_w, quat_wxyz);

//         // Motor thrust axes: all point +z in body frame
//         motor_axis_.setZero();
//         for (int i = 0; i < N_ROTORS; ++i) motor_axis_.row(i) << 0.0, 0.0, 1.0;

//         const double a  = arm_length_ * std::sqrt(2.0) / 2.0;
//         const double zu = +0.5 * z_sep_;
//         const double zl = -0.5 * z_sep_;

//         // X8 layout: 4 arms, each arm = upper (even idx) + lower (odd idx)
//         // Body frame: x forward, y left, z up
//         //   Pair 0  front-right
//         motor_pos_.row(0) <<  a, -a, zu;
//         motor_pos_.row(1) <<  a, -a, zl;
//         //   Pair 1  rear-left
//         motor_pos_.row(2) << -a,  a, zu;
//         motor_pos_.row(3) << -a,  a, zl;
//         //   Pair 2  front-left
//         motor_pos_.row(4) <<  a,  a, zu;
//         motor_pos_.row(5) <<  a,  a, zl;
//         //   Pair 3  rear-right
//         motor_pos_.row(6) << -a, -a, zu;
//         motor_pos_.row(7) << -a, -a, zl;

//         motor_rpm_.setZero();
//         motor_rpm_cmd_.setZero();
//     }

//     // ---- Configuration ----
//     void setGeometry(double arm_length, double z_sep) {
//         arm_length_ = arm_length;
//         z_sep_ = z_sep;
//     }
//     void setLimits(double min_rpm, double max_rpm) {
//         min_rpm_ = min_rpm;
//         max_rpm_ = max_rpm;
//     }
//     void setMotorTimeConstant(double tau) { motor_tau_ = tau; }
//     void setDragCoeffs(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
//         drag_linear_body_ = linear;
//         drag_angular_body_ = angular;
//     }

//     // ---- Motor commands ----
//     void setRPMCmd(const Eigen::Matrix<double, N_ROTORS, 1>& cmd) {
//         motor_rpm_cmd_ = cmd;
//         for (int i = 0; i < N_ROTORS; ++i)
//             motor_rpm_cmd_(i) = std::clamp(motor_rpm_cmd_(i), min_rpm_, max_rpm_);
//     }

//     Eigen::Matrix<double, N_ROTORS, 1> getRPM() const { return motor_rpm_; }

//     // ---- Access to aero model (for allocator) ----
//     const CoaxialRotorModel& aeroModel() const { return aero_model_; }

//     // ---- Main simulation step ----
//     void step_forward(double dt)
//     {
//         if (dt <= 1e-6) return;

//         updateMotorDynamics(dt);

//         const Eigen::Vector3d z_b(0.0, 0.0, 1.0);

//         Eigen::Vector3d thrust_b = Eigen::Vector3d::Zero();
//         Eigen::Vector3d moment_b = Eigen::Vector3d::Zero();

//         // [BUG-1 FIX] Compute per-PAIR aerodynamics from CFD model
//         for (int p = 0; p < N_PAIRS; ++p) {
//             const int iu = PAIRS[p].first;
//             const int il = PAIRS[p].second;

//             const double rpm_u = motor_rpm_(iu);
//             const double rpm_l = motor_rpm_(il);

//             // Query CFD-based RBF model for this pair
//             auto aero = aero_model_.query(rpm_u, rpm_l);

//             // Thrust forces along body +z
//             const Eigen::Vector3d F_upper = aero.T_u * z_b;
//             const Eigen::Vector3d F_lower = aero.T_l * z_b;
//             thrust_b += F_upper + F_lower;

//             // Moments from thrust × position (roll/pitch)
//             const Eigen::Vector3d r_u = motor_pos_.row(iu).transpose();
//             const Eigen::Vector3d r_l = motor_pos_.row(il).transpose();
//             moment_b += r_u.cross(F_upper) + r_l.cross(F_lower);

//             // [BUG-3 FIX] Yaw reaction torque — CFD Q is already signed.
//             // Apply pair_yaw_sign to account for alternating spin directions.
//             // Pairs 0,2 match CFD convention; pairs 1,3 are flipped.
//             const double ys = PAIR_YAW_SIGN[p];
//             moment_b += ys * aero.Q_u * z_b;   // Q_u > 0 from CFD
//             moment_b += ys * aero.Q_l * z_b;   // Q_l < 0 from CFD
//         }


//         const Eigen::Vector3d vel_b = R_bw_.transpose() * vel_w_;

//         const double v_x = vel_b.x();
//         const double v_z = vel_b.z();

//         const double V = v_x;

//         const double AoA_rad = (std::abs(v_x) > 1e-3)
//                             ? std::atan2(-v_z, std::abs(v_x))
//                             : 0.0;

//         const double AoA_deg = AoA_rad * 180.0 / M_PI;

//         const double V_q = std::clamp(
//             V,
//             FuselageAeroModel::V_MIN - 5.0,
//             FuselageAeroModel::V_MAX + 5.0
//         );

//         const double AoA_q = std::clamp(
//             AoA_deg,
//             FuselageAeroModel::AOA_MIN - 5.0,
//             FuselageAeroModel::AOA_MAX + 5.0
//         );

//         const auto fa = fuselage_aero_.query(V_q, AoA_q);

//         Eigen::Vector3d fuselage_force_b  = fa.force;
//         Eigen::Vector3d fuselage_torque_b = fa.torque;
    

//         // Gravity in body frame
//         const Eigen::Vector3d gravity_b =
//             mass_ * R_bw_.transpose() * Eigen::Vector3d(0.0, 0.0, -g_);

//         // Translational drag (body frame)
//         const Eigen::Vector3d drag_b = -drag_linear_body_.cwiseProduct(vel_b);

//         // Angular damping (body frame)
//         const Eigen::Vector3d aero_damp =
//             -drag_angular_body_.cwiseProduct(omega_b_);

//         // Assemble
//         force_b_  = thrust_b + gravity_b + drag_b + fuselage_force_b;
//         torque_b_ = moment_b + aero_damp + fuselage_torque_b;

//         force_w_  = R_bw_ * force_b_;
//         torque_w_ = R_bw_ * torque_b_;

//         step(dt);
//     }

// private:
//     void updateMotorDynamics(double dt) {
//         const double alpha = std::min(1.0, dt / std::max(1e-4, motor_tau_));
//         motor_rpm_ += alpha * (motor_rpm_cmd_ - motor_rpm_);
//         for (int i = 0; i < N_ROTORS; ++i)
//             motor_rpm_(i) = std::clamp(motor_rpm_(i), min_rpm_, max_rpm_);
//     }
// };


#pragma once
/**
 * coaxial_x8_dynamics.hpp  —  X8 coaxial multirotor dynamics (with wind disturbance)
 *
 * 风扰动模型 (WindModel)：
 *   v_wind_world = v_mean + v_turb + v_gust
 *   - v_mean : 用户设定的平均风（世界系，例如东风 5 m/s）
 *   - v_turb : Ornstein-Uhlenbeck 过程产生的湍流分量（带相关时间）
 *   - v_gust : 离散阵风（"1-cos" 形状，间隔随机触发）
 *
 *   机体感受到的相对气流：
 *      v_air_body = R_bw^T * (v_world - v_wind_world)
 *   把它喂给 fuselage_aero_model 做查询，从而风以物理一致的方式影响气动。
 *   旋翼的诱导速度模型这里没有耦合（CFD 表是在静止空气中标定的），
 *   如果以后要做更精细的，可以再把 v_air_body 投到桨盘上。
 */
#include <array>
#include <algorithm>
#include <cmath>
#include <random>
#include "dynamics.hpp"
#include "coaxial_rotor_model.hpp"
#include "fuselage_aero_model.hpp"

// ============================================================================
//  WindModel — 平均风 + OU 湍流 + 离散阵风
// ============================================================================
class WindModel
{
public:
    struct Params {
        // 平均风（世界系，m/s）。例如 (5, 0, 0) 表示沿世界 +x 5 m/s 的稳风。
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        // 湍流强度（每个分量的稳态标准差，m/s）
        Eigen::Vector3d turb_sigma = Eigen::Vector3d(0.5, 0.5, 0.3);

        // 湍流相关时间 [s]（越大越慢，越小越碎）
        double turb_tau = 1.0;

        // 阵风
        bool   gust_enable    = true;
        double gust_amp_mean  = 10.0;   // 阵风幅值的均值 [m/s]
        double gust_amp_sigma = 1.0;   // 阵风幅值的随机标准差 [m/s]
        double gust_duration  = 1.5;   // 单次阵风持续时间 [s]
        double gust_rate      = 0.05;  // 每秒触发概率（泊松近似）

        // 随机种子（同一个种子 → 同一段风，便于复现）
        uint32_t seed = 0xC0FFEEu;
    };

    WindModel() : WindModel(Params()) {}

    explicit WindModel(const Params& p) : p_(p), rng_(p.seed)
    {
        v_turb_.setZero();
        gust_active_   = false;
        gust_t_        = 0.0;
        gust_dir_.setZero();
        gust_amp_      = 0.0;
    }

    void setParams(const Params& p) {
        p_ = p;
        rng_.seed(p.seed);
        v_turb_.setZero();
        gust_active_ = false;
    }

    // 推进风模型一步，返回当前风速（世界系，m/s）
    Eigen::Vector3d step(double dt)
    {
        if (dt <= 0.0) return p_.mean + v_turb_ + currentGust();

        // ---- 1. OU 湍流: dv = -v/tau dt + sigma * sqrt(2/tau) dW ----
        const double tau = std::max(1e-3, p_.turb_tau);
        const double a   = std::exp(-dt / tau);
        // 离散等价的 OU：v <- a*v + sqrt(1-a^2) * sigma * N(0,1)
        const double s   = std::sqrt(std::max(0.0, 1.0 - a * a));
        std::normal_distribution<double> N01(0.0, 1.0);
        for (int i = 0; i < 3; ++i) {
            v_turb_(i) = a * v_turb_(i) + s * p_.turb_sigma(i) * N01(rng_);
        }

        // ---- 2. 阵风：若未激活，按泊松率随机触发 ----
        if (p_.gust_enable) {
            if (!gust_active_) {
                // 每步触发概率 = 1 - exp(-rate * dt)
                std::uniform_real_distribution<double> U(0.0, 1.0);
                const double p_trigger = 1.0 - std::exp(-std::max(0.0, p_.gust_rate) * dt);
                if (U(rng_) < p_trigger) {
                    triggerGust();
                }
            } else {
                gust_t_ += dt;
                if (gust_t_ >= p_.gust_duration) {
                    gust_active_ = false;
                    gust_t_      = 0.0;
                }
            }
        }

        return p_.mean + v_turb_ + currentGust();
    }

    // 在不推进时间的情况下读取当前风速
    Eigen::Vector3d current() const {
        return p_.mean + v_turb_ + currentGust();
    }

    bool   gustActive() const { return gust_active_; }
    double gustPhase()  const { return gust_active_ ? gust_t_ / std::max(1e-6, p_.gust_duration) : 0.0; }

private:
    Params           p_;
    std::mt19937     rng_;
    Eigen::Vector3d  v_turb_;

    // 阵风状态
    bool             gust_active_;
    double           gust_t_;
    Eigen::Vector3d  gust_dir_;
    double           gust_amp_;

    void triggerGust()
    {
        std::normal_distribution<double> N(0.0, 1.0);
        std::uniform_real_distribution<double> U(0.0, 1.0);

        // 随机方向（球面均匀采样）
        const double u1 = U(rng_);
        const double u2 = U(rng_);
        const double theta = 2.0 * M_PI * u1;
        const double z     = 2.0 * u2 - 1.0;
        const double r     = std::sqrt(std::max(0.0, 1.0 - z * z));
        gust_dir_ << r * std::cos(theta), r * std::sin(theta), z;

        // 随机幅值（截断为非负）
        gust_amp_ = std::max(0.0, p_.gust_amp_mean + p_.gust_amp_sigma * N(rng_));

        gust_active_ = true;
        gust_t_      = 0.0;
    }

    // "1-cos" 形状的阵风：g(t) = 0.5 * amp * (1 - cos(2π t / D)) * dir
    Eigen::Vector3d currentGust() const
    {
        if (!gust_active_) return Eigen::Vector3d::Zero();
        const double phase = std::clamp(gust_t_ / std::max(1e-6, p_.gust_duration), 0.0, 1.0);
        const double shape = 0.5 * (1.0 - std::cos(2.0 * M_PI * phase));
        return gust_amp_ * shape * gust_dir_;
    }
};

// ============================================================================
//  X8 dynamics
// ============================================================================
class coaxial_x8_dynamics : public dynamics
{
public:
    static constexpr int N_ROTORS = 8;
    static constexpr int N_PAIRS  = 4;

    static constexpr std::array<std::pair<int,int>, N_PAIRS> PAIRS = {
        {{0,1}, {2,3}, {4,5}, {6,7}}
    };

    static constexpr std::array<double, N_PAIRS> PAIR_YAW_SIGN = {+1.0, -1.0, +1.0, -1.0};

private:
    CoaxialRotorModel aero_model_;
    FuselageAeroModel fuselage_aero_;
    WindModel         wind_;                 // <— 新增

    Eigen::Matrix<double, N_ROTORS, 1> motor_rpm_cmd_ = Eigen::Matrix<double, N_ROTORS, 1>::Zero();
    Eigen::Matrix<double, N_ROTORS, 1> motor_rpm_     = Eigen::Matrix<double, N_ROTORS, 1>::Zero();

    Eigen::Matrix<double, N_ROTORS, 3> motor_pos_;
    Eigen::Matrix<double, N_ROTORS, 3> motor_axis_;

    double arm_length_    = 1.65;
    double z_sep_         = 0.40;
    double min_rpm_       = CoaxialRotorModel::rpmMin();
    double max_rpm_       = CoaxialRotorModel::rpmMax();
    double motor_tau_     = 0.03;
    double g_             = 9.81;

    Eigen::Vector3d drag_linear_body_  = Eigen::Vector3d(0.08, 0.08, 0.12);
    Eigen::Vector3d drag_angular_body_ = Eigen::Vector3d(0.0008, 0.0008, 0.0012);

    // 当前世界系风速（每步 step_forward 更新一次，便于外部读取/记录）
    Eigen::Vector3d wind_w_ = Eigen::Vector3d::Zero();
    bool            wind_enabled_ = true;

public:
    using dynamics::dynamics;

    void init(const Eigen::Vector3d& p_w, const Eigen::Vector4d& quat_wxyz)
    {
        initialize(p_w, quat_wxyz);

        motor_axis_.setZero();
        for (int i = 0; i < N_ROTORS; ++i) motor_axis_.row(i) << 0.0, 0.0, 1.0;

        const double a  = arm_length_ * std::sqrt(2.0) / 2.0;
        const double zu = +0.5 * z_sep_;
        const double zl = -0.5 * z_sep_;

        motor_pos_.row(0) <<  a, -a, zu;
        motor_pos_.row(1) <<  a, -a, zl;
        motor_pos_.row(2) << -a,  a, zu;
        motor_pos_.row(3) << -a,  a, zl;
        motor_pos_.row(4) <<  a,  a, zu;
        motor_pos_.row(5) <<  a,  a, zl;
        motor_pos_.row(6) << -a, -a, zu;
        motor_pos_.row(7) << -a, -a, zl;

        motor_rpm_.setZero();
        motor_rpm_cmd_.setZero();
        wind_w_.setZero();
    }

    // ---- Configuration ----
    void setGeometry(double arm_length, double z_sep) { arm_length_ = arm_length; z_sep_ = z_sep; }
    void setLimits(double min_rpm, double max_rpm)    { min_rpm_ = min_rpm; max_rpm_ = max_rpm; }
    void setMotorTimeConstant(double tau)             { motor_tau_ = tau; }
    void setDragCoeffs(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
        drag_linear_body_  = linear;
        drag_angular_body_ = angular;
    }

    // ---- Wind configuration ----
    void setWindParams(const WindModel::Params& wp) { wind_.setParams(wp); }
    void enableWind(bool on)                        { wind_enabled_ = on; }
    Eigen::Vector3d getWindWorld() const            { return wind_w_; }
    bool            isGustActive() const            { return wind_.gustActive(); }

    // ---- Motor commands ----
    void setRPMCmd(const Eigen::Matrix<double, N_ROTORS, 1>& cmd) {
        motor_rpm_cmd_ = cmd;
        for (int i = 0; i < N_ROTORS; ++i)
            motor_rpm_cmd_(i) = std::clamp(motor_rpm_cmd_(i), min_rpm_, max_rpm_);
    }

    Eigen::Matrix<double, N_ROTORS, 1> getRPM() const { return motor_rpm_; }

    const CoaxialRotorModel& aeroModel() const { return aero_model_; }

    // ---- Main simulation step ----
    void step_forward(double dt)
    {
        if (dt <= 1e-6) return;

        updateMotorDynamics(dt);

        // ---- 推进风模型 ----
        if (wind_enabled_) {
            wind_w_ = wind_.step(dt);
        } else {
            wind_w_.setZero();
        }

        const Eigen::Vector3d z_b(0.0, 0.0, 1.0);

        // ---- 机体相对气流（相对风，不是地速）：先算好，旋翼与机身共用 ----
        // v_air_world = v_world - v_wind_world
        const Eigen::Vector3d vel_air_w = vel_w_ - wind_w_;
        const Eigen::Vector3d vel_air_b = R_bw_.transpose() * vel_air_w;

        const double v_x = vel_air_b.x();
        const double v_z = vel_air_b.z();

        const double V = v_x;                                   // 机体前向气速（带符号）
        // 桨盘（轴沿 +z）感受到的掠流为机体水平面内的气速幅值
        const double V_edgewise = std::hypot(vel_air_b.x(), vel_air_b.y());
        const double AoA_rad = (std::abs(v_x) > 1e-3)
                            ? std::atan2(-v_z, std::abs(v_x))
                            : 0.0;
        const double AoA_deg = AoA_rad * 180.0 / M_PI;

        Eigen::Vector3d thrust_b = Eigen::Vector3d::Zero();
        Eigen::Vector3d moment_b = Eigen::Vector3d::Zero();

        // ---- 旋翼气动（CFD 表 + 前飞/迎角修正，已与相对气流耦合）----
        for (int p = 0; p < N_PAIRS; ++p) {
            const int iu = PAIRS[p].first;
            const int il = PAIRS[p].second;

            const double rpm_u = motor_rpm_(iu);
            const double rpm_l = motor_rpm_(il);

            auto aero = aero_model_.queryWithInflow(rpm_u, rpm_l, V_edgewise, AoA_deg);

            const Eigen::Vector3d F_upper = aero.T_u * z_b;
            const Eigen::Vector3d F_lower = aero.T_l * z_b;
            thrust_b += F_upper + F_lower;

            const Eigen::Vector3d r_u = motor_pos_.row(iu).transpose();
            const Eigen::Vector3d r_l = motor_pos_.row(il).transpose();
            moment_b += r_u.cross(F_upper) + r_l.cross(F_lower);

            const double ys = PAIR_YAW_SIGN[p];
            moment_b += ys * aero.Q_u * z_b;
            moment_b += ys * aero.Q_l * z_b;
        }

        // ---- 机身气动：复用上面已算好的相对气流 V / AoA ----
        const double V_q = std::clamp(
            V, FuselageAeroModel::V_MIN - 5.0, FuselageAeroModel::V_MAX + 5.0);
        const double AoA_q = std::clamp(
            AoA_deg, FuselageAeroModel::AOA_MIN - 5.0, FuselageAeroModel::AOA_MAX + 5.0);

        const auto fa = fuselage_aero_.query(V_q, AoA_q);
        Eigen::Vector3d fuselage_force_b  = fa.force;
        Eigen::Vector3d fuselage_torque_b = fa.torque;

        // 重力
        const Eigen::Vector3d gravity_b =
            mass_ * R_bw_.transpose() * Eigen::Vector3d(0.0, 0.0, -g_);

        // 平动阻尼也用相对风速来算（这样静止时风也能把机体推走）
        const Eigen::Vector3d drag_b = -drag_linear_body_.cwiseProduct(vel_air_b);

        // 角阻尼
        const Eigen::Vector3d aero_damp =
            -drag_angular_body_.cwiseProduct(omega_b_);

        force_b_  = thrust_b + gravity_b + drag_b + fuselage_force_b;
        torque_b_ = moment_b + aero_damp + fuselage_torque_b;

        force_w_  = R_bw_ * force_b_;
        torque_w_ = R_bw_ * torque_b_;

        step(dt);
    }

private:
    void updateMotorDynamics(double dt) {
        const double alpha = std::min(1.0, dt / std::max(1e-4, motor_tau_));
        motor_rpm_ += alpha * (motor_rpm_cmd_ - motor_rpm_);
        for (int i = 0; i < N_ROTORS; ++i)
            motor_rpm_(i) = std::clamp(motor_rpm_(i), min_rpm_, max_rpm_);
    }
};