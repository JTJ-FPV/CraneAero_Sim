#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include <Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>

#include "dynamics.hpp"
#include "coaxial_rotor_model.hpp"
#include "coaxial_x8_dynamics.hpp"
#include "coaxial_x8_allocator.hpp"
#include "coaxial_x8_allocator_qp.hpp"
#include "coaxial_x8_controller.hpp"


// ===================== 控制模式 =====================
//
// 状态机：
//
//      ┌──────────────┐    btn8 上升沿     ┌───────────────┐
//      │ RC_CONTROL   │ ─────────────────▶ │ CMD_WAITING   │
//      │ (遥控器)     │ ◀───────────────── │ (等待首个指令)│
//      └──────────────┘   btn8 上升沿       └───────┬───────┘
//             ▲                                    │
//             │ btn8 上升沿                        │ 首个 cmd_pose
//             │                                    │ 或 planning_cmd
//             │              ┌─────────────────────┴────────────────────┐
//             │              ▼                                          ▼
//             │     ┌─────────────────┐                       ┌─────────────────┐
//             └─────│ POSITION_CONTROL│                       │ PLANNING_CONTROL│
//                   │ (cmd_pose 锁)   │                       │ (planning 锁)   │
//                   └─────────────────┘                       └─────────────────┘
//
// "先到为准"：CMD_WAITING 收到的第一条指令决定锁哪边，之后另一边的消息被忽略。
// 任何指令模式按下按钮都会回到 RC_CONTROL。
//
enum ControlMode {
    RC_CONTROL       = 0,
    CMD_WAITING      = 1,
    POSITION_CONTROL = 2,
    PLANNING_CONTROL = 3,
};

static const int MODE_TOGGLE_BUTTON = 8;        // 第 8 号按钮（从 0 开始）

ControlMode current_control_mode = RC_CONTROL;
bool        snap_setpoint_to_current = false;   // 进入 CMD_WAITING 时锁定到当前位姿

ros::Publisher control_RPM_pub, odom_pub, imu_pub;
ros::Publisher pos_error, vel_error;
ros::Publisher wind_pub, gust_pub;
Eigen::Vector3d pos_des, vel_des, acc_des;
geometry_msgs::PoseStamped pose_cmd;
double yaw_des = 0.0;

RcInput rc_input;


// ===================== 回调 =====================
void cmd_callback(const geometry_msgs::PoseStamped& msg)
{
    pose_cmd = msg;

    // 仅在等待指令时被首条 cmd_pose 锁定
    if (current_control_mode == CMD_WAITING) {
        current_control_mode = POSITION_CONTROL;
        rc_input.active = false;
        ROS_WARN("[Mode] CMD_WAITING -> POSITION_CONTROL  (cmd_pose arrived first)");
    }
    // 其他状态下只缓存 pose_cmd，不切模式
}

void planning_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
    // RC 模式或已锁定到 cmd_pose 模式，直接忽略 planning_cmd
    if (current_control_mode == RC_CONTROL ||
        current_control_mode == POSITION_CONTROL) {
        return;
    }

    // CMD_WAITING 收到的第一条 planning_cmd → 锁为 PLANNING_CONTROL
    if (current_control_mode == CMD_WAITING) {
        current_control_mode = PLANNING_CONTROL;
        rc_input.active = false;
        ROS_WARN("[Mode] CMD_WAITING -> PLANNING_CONTROL (planning_cmd arrived first)");
    }

    pos_des = Eigen::Vector3d(cmd->position.x,     cmd->position.y,     cmd->position.z);
    vel_des = Eigen::Vector3d(cmd->velocity.x,     cmd->velocity.y,     cmd->velocity.z);
    acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    yaw_des = cmd->yaw;

    ROS_INFO_THROTTLE(1.0,
        "Received planning command: pos (%.2f, %.2f, %.2f), vel (%.2f, %.2f, %.2f), acc (%.2f, %.2f, %.2f)",
        pos_des.x(), pos_des.y(), pos_des.z(),
        vel_des.x(), vel_des.y(), vel_des.z(),
        acc_des.x(), acc_des.y(), acc_des.z());
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    // ---- 1. 摇杆轴 ----
    if (msg->axes.size() > 4) {
        rc_input.pitch_stick    =  msg->axes[4];   // 右摇杆 上下 → pitch
        rc_input.roll_stick     = -msg->axes[3];   // 右摇杆 左右 → roll
        rc_input.throttle_stick =  msg->axes[1];   // 左摇杆 上下 → throttle
        rc_input.yaw_stick      =  msg->axes[0];   // 左摇杆 左右 → yaw
    }

    // ---- 2. 模式切换按钮（第 8 号）的上升沿检测 ----
    static bool prev_btn = false;
    const bool curr_btn = (msg->buttons.size() > MODE_TOGGLE_BUTTON) &&
                          (msg->buttons[MODE_TOGGLE_BUTTON] == 1);

    if (curr_btn && !prev_btn) {
        // 上升沿
        if (current_control_mode == RC_CONTROL) {
            // RC -> 等待指令：把 setpoint 在 main 循环里锁到当前位姿，避免飞机突然"飞向旧目标"
            current_control_mode = CMD_WAITING;
            rc_input.active = false;
            snap_setpoint_to_current = true;
            ROS_WARN("[Mode] btn%d ↑ : RC_CONTROL -> CMD_WAITING  (waiting for first cmd…)",
                     MODE_TOGGLE_BUTTON);
        } else {
            // 任何指令模式 -> RC
            current_control_mode = RC_CONTROL;
            rc_input.active = true;
            ROS_WARN("[Mode] btn%d ↑ : -> RC_CONTROL", MODE_TOGGLE_BUTTON);
        }
    }
    prev_btn = curr_btn;

    // ---- 3. rc_input.active 严格跟随当前模式 ----
    rc_input.active = (current_control_mode == RC_CONTROL);
}


static constexpr double R_DISK     = 1.65;     // X8 arm length [m]
static constexpr double F_BODY     = 0.85;     // mass fraction at centre
static constexpr double R_BODY     = 0.30;     // central body radius [m]
static constexpr double H_BODY     = 0.50;     // central body height [m]
static constexpr double Z_SEP_HALF = 0.20;     // half upper/lower rotor separation [m]

// √(1/2) hard-coded for constexpr correctness (std::sqrt is not constexpr in C++17)
static constexpr double SQRT_HALF  = 0.70710678118654752;
static constexpr double A_ARM      = R_DISK * SQRT_HALF;     // 1.1668 m, off-axis distance

static constexpr double KJX =
      (1.0 - F_BODY) * (A_ARM * A_ARM + Z_SEP_HALF * Z_SEP_HALF)
    +  F_BODY        * (3.0 * R_BODY * R_BODY + H_BODY * H_BODY) / 12.0;

static constexpr double KJY = KJX;

static constexpr double KJZ =
      (1.0 - F_BODY) * 2.0 * A_ARM * A_ARM
    +  F_BODY        * R_BODY * R_BODY / 2.0;

static Eigen::Matrix3d inertiaForMass(double m,
                                      double kx = KJX,
                                      double ky = KJY,
                                      double kz = KJZ)
{
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    J(0, 0) = kx * m;
    J(1, 1) = ky * m;
    J(2, 2) = kz * m;
    return J;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crane_control_node");
    ros::NodeHandle nh("~");
    double init_x, init_y, init_z, mass;
    double simulation_rate;
    std::string quad_name;
    nh.param("mass", mass, 300.0);
    nh.param("init_state_x", init_x, 0.0);
    nh.param("init_state_y", init_y, 0.0);
    nh.param("init_state_z", init_z, 1.0);
    nh.param("simulation_rate", simulation_rate, 200.0);
    nh.param("coaxial_name", quad_name, std::string("coaxial_1"));

    odom_pub        = nh.advertise<nav_msgs::Odometry>("odom", 100);
    imu_pub         = nh.advertise<sensor_msgs::Imu>("imu", 10);
    control_RPM_pub = nh.advertise<std_msgs::Float32MultiArray>("cmd_rpm", 100);
    pos_error       = nh.advertise<geometry_msgs::Point>("position_error", 100);
    vel_error       = nh.advertise<geometry_msgs::Point>("velocity_error", 100);
    wind_pub        = nh.advertise<geometry_msgs::Vector3Stamped>("wind", 10);
    gust_pub        = nh.advertise<std_msgs::Bool>("wind_gust_active", 10);

    ros::Subscriber cmd_sub          = nh.subscribe("cmd_pose",     100, &cmd_callback,          ros::TransportHints().tcpNoDelay());
    ros::Subscriber planning_cmd_sub = nh.subscribe("planning_cmd", 100, &planning_cmd_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber joy_sub          = nh.subscribe("/joy",         10,  &joyCallback,           ros::TransportHints().tcpNoDelay());

    pos_des = Eigen::Vector3d(init_x, init_y, init_z);
    vel_des = Eigen::Vector3d::Zero();
    acc_des = Eigen::Vector3d::Zero();

    CoaxialX8Controller ctrl(simulation_rate);

    // Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    // I(0,0) = 500; I(1,1) = 500; I(2,2) = 800;

    const double R = 1.65;
    // Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    // I(0,0) = mass * R * R / 4.0;
    // I(1,1) = mass * R * R / 4.0;
    // I(2,2) = mass * R * R / 2.0;
    Eigen::Matrix3d I = inertiaForMass(mass);

    
    const double g = 9.81;
    CoaxialX8Controller::Params cp;
    cp.mass    = mass;
    cp.inertia = I;
    cp.g       = g;
    ctrl.setParams(cp);

    RcConfig rc_cfg;
    rc_cfg.deadzone     = 0.05;
    rc_cfg.expo         = 0.3;
    rc_cfg.max_vel_xy   = 3.0;
    rc_cfg.max_vel_z    = 2.0;
    rc_cfg.max_yaw_rate = 25.0 * M_PI / 180.0;
    ctrl.setRcConfig(rc_cfg);

    CoaxialX8Controller::Gains cg;
    // cg.pos_stable_time = 4.7;
    cg.pos_stable_time = 3.2;
    cg.alt_stable_time = 3.8;
    cg.att_stable_time = 0.9;
    // cg.att_stable_time = 1.0;
    cg.att_damping     = 0.9;
    cg.yaw_rel_kp      = 1.0;
    cg.yaw_rel_kd      = 1.0;
    ctrl.setGains(cg);

    // ------------- Allocator -------------
    CoaxialX8Allocator::Params ap;
    ap.arm_length = R_DISK;
    CoaxialX8Allocator allocator(ap);
    CoaxialX8AllocatorQP::Params ap_qp;
    ap_qp.arm_length = R_DISK;
    ap_qp.w_thrust   = 100.0;
    ap_qp.w_moment   = 10.0;
    ap_qp.w_yaw      = 1.0;
    ap_qp.verbose    = false;
    CoaxialX8AllocatorQP alloc(ap_qp);

    CoaxialX8Controller::Limits cl;
    cl.max_thrust_N     = 4.0 * alloc.getTmax();
    cl.max_torque_xy_Nm = 3000.0;
    cl.max_torque_z_Nm  = 116.0;
    const double cos_tilt_min = mass * g / cl.max_thrust_N;
    if (cos_tilt_min >= 1.0) {
        ROS_FATAL("TWR < 1 — Cannot take off!");
    }
    cl.max_tilt_rad     = std::acos(cos_tilt_min) * 0.5;
    cl.max_acc_z_up     =  1.5;
    cl.max_acc_z_down   = -1.0;
    cl.max_acc_xy       =  1.5;
    // cl.max_vel_err      =  5.0;
    // cl.max_pos_err      =  5.0;
    cl.max_yaw_rate_rad = 25.0 * M_PI / 180.0;
    ctrl.setLimits(cl);

    // ------------- Dynamics -------------
    coaxial_x8_dynamics sim(mass, I);
    sim.setGeometry(1.65, 0.40);
    sim.init(Eigen::Vector3d(init_x, init_y, init_z),
             Eigen::Vector4d(1, 0, 0, 0));

    // ------------- Wind disturbance ------
    {
        WindModel::Params wp;
        bool wind_enable;
        nh.param("wind/enable",         wind_enable,        true);

        double mx, my, mz;
        nh.param("wind/mean_x",         mx, 0.0);
        nh.param("wind/mean_y",         my, 0.0);
        nh.param("wind/mean_z",         mz, 0.0);
        wp.mean = Eigen::Vector3d(mx, my, mz);

        double sxy, sz;
        nh.param("wind/turb_sigma_xy",  sxy, 0.5);
        nh.param("wind/turb_sigma_z",   sz,  0.3);
        wp.turb_sigma = Eigen::Vector3d(sxy, sxy, sz);

        nh.param("wind/turb_tau",       wp.turb_tau,       2.0);
        nh.param("wind/gust_enable",    wp.gust_enable,    true);
        nh.param("wind/gust_amp_mean",  wp.gust_amp_mean,  3.0);
        nh.param("wind/gust_amp_sigma", wp.gust_amp_sigma, 1.0);
        nh.param("wind/gust_duration",  wp.gust_duration,  1.5);
        nh.param("wind/gust_rate",      wp.gust_rate,      0.05);

        int seed_i = static_cast<int>(0xC0FFEE);
        nh.param("wind/seed",           seed_i, seed_i);
        wp.seed = static_cast<uint32_t>(seed_i);

        sim.setWindParams(wp);
        sim.enableWind(wind_enable);

        ROS_INFO("[Wind] enable=%d  mean=(%.1f,%.1f,%.1f) m/s  "
                 "sigma=(%.2f,%.2f,%.2f)  tau=%.1fs  gust(amp=%.1f±%.1f, dur=%.1fs, rate=%.2f/s)",
                 (int)wind_enable, mx, my, mz, sxy, sxy, sz,
                 wp.turb_tau, wp.gust_amp_mean, wp.gust_amp_sigma,
                 wp.gust_duration, wp.gust_rate);
    }

    ros::Rate rate(simulation_rate);
    ros::Time last_time = ros::Time::now();

    const double mg = mass * g;
    for (double Mz = 0; Mz <= 200; Mz += 1) {
        auto r = alloc.allocate(mg, Eigen::Vector3d(0, 0, Mz));
        ROS_INFO("Mz_cmd=%6.1f  Mz_ach=%6.1f  dF=%+6.1f  qp_ok=%d",
                 Mz, r.wrench_ach(3), r.wrench_ach(0) - mg, (int)r.qp_success);
    }

    while (ros::ok())
    {
        ros::spinOnce();

        // ---- 反馈 ----
        Eigen::Vector4d q = sim.getQuat();          // (w,x,y,z)
        Eigen::Quaterniond q_wb(q(0), q(1), q(2), q(3));
        ctrl.setFeedback(sim.getPos(), sim.getVel(),
                         q_wb, sim.getAngularVelBody());

        // ---- 离开 RC 时复位 latch ----
        static ControlMode last_mode = RC_CONTROL;
        if (last_mode == RC_CONTROL && current_control_mode != RC_CONTROL) {
            ctrl.resetRcLatch();
        }
        last_mode = current_control_mode;

        // ---- 进入 CMD_WAITING 那一帧：把 setpoint 锁到当前位姿，避免飞机飞向旧目标 ----
        if (snap_setpoint_to_current) {
            pos_des = sim.getPos();
            vel_des.setZero();
            acc_des.setZero();
            const Eigen::Vector3d xb = q_wb.toRotationMatrix() * Eigen::Vector3d::UnitX();
            yaw_des = std::atan2(xb.y(), xb.x());
            snap_setpoint_to_current = false;
            ROS_WARN("[Mode] hold setpoint @ (%.2f, %.2f, %.2f), yaw=%.2f",
                     pos_des.x(), pos_des.y(), pos_des.z(), yaw_des);
        }

        // ---- 模式分发 ----
        switch (current_control_mode)
        {
            case POSITION_CONTROL:
            {
                pos_des << pose_cmd.pose.position.x,
                           pose_cmd.pose.position.y,
                           pose_cmd.pose.position.z;
                Eigen::Quaterniond q_des(pose_cmd.pose.orientation.w,
                                         pose_cmd.pose.orientation.x,
                                         pose_cmd.pose.orientation.y,
                                         pose_cmd.pose.orientation.z);
                const Eigen::Vector3d xb = q_des.matrix() * Eigen::Vector3d::UnitX();
                yaw_des = std::atan2(xb.y(), xb.x());
                vel_des.setZero();
                acc_des.setZero();
                ctrl.setSetpoint(pos_des, vel_des, acc_des, yaw_des);
                break;
            }
            case PLANNING_CONTROL:
                // pos_des/vel_des/acc_des/yaw_des 已在 callback 写入
                ctrl.setSetpoint(pos_des, vel_des, acc_des, yaw_des);
                break;

            case CMD_WAITING:
                // 等待首条指令——保持 setpoint 不动（已 snap 到当前位姿）
                ctrl.setSetpoint(pos_des, vel_des, acc_des, yaw_des);
                break;

            case RC_CONTROL:
            default:
                ctrl.updateFromRC(rc_input);
                break;
        }
        
        auto wrench = ctrl.run();

        auto rpm8 = alloc.allocate(wrench.thrust_body_z, wrench.torque_body);
        sim.setRPMCmd(rpm8.rpm);

        Eigen::Vector4d want;
        want << wrench.thrust_body_z,
                wrench.torque_body(0), wrench.torque_body(1), wrench.torque_body(2);
        Eigen::Vector4d diff = rpm8.wrench_ach - want;

        // ROS_INFO_THROTTLE(0.05,
        //     "ALLOC qp=%d inv=%d it=%d res=%.3f | "
        //     "dF=%+.1f dMx=%+.2f dMy=%+.2f dMz=%+.2f | "
        //     "RPM %.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f",
        //     (int)rpm8.qp_success, (int)rpm8.invert_ok, rpm8.qp_iters, rpm8.residual,
        //     diff(0), diff(1), diff(2), diff(3),
        //     rpm8.rpm(0),rpm8.rpm(1),rpm8.rpm(2),rpm8.rpm(3),
        //     rpm8.rpm(4),rpm8.rpm(5),rpm8.rpm(6),rpm8.rpm(7));

        ros::Time now_time = ros::Time::now();
        double dt = (now_time - last_time).toSec();
        last_time = now_time;

        if (dt <= 0.0 || dt > 0.02) {
            dt = 1.0 / simulation_rate;
        }

        sim.step_forward(dt);

        // ---- 发布 风速 / 阵风状态 ----
        {
            const Eigen::Vector3d w = sim.getWindWorld();
            geometry_msgs::Vector3Stamped wm;
            wm.header.stamp    = now_time;
            wm.header.frame_id = "world";
            wm.vector.x = w.x();
            wm.vector.y = w.y();
            wm.vector.z = w.z();
            wind_pub.publish(wm);

            std_msgs::Bool gm;
            gm.data = sim.isGustActive();
            gust_pub.publish(gm);
        }

        // ---- 发布 RPM ----
        std_msgs::Float32MultiArray rpm_array;
        rpm_array.data.clear();
        for (size_t i = 0; i < 8; ++i)
            rpm_array.data.emplace_back(static_cast<float>(rpm8.rpm(i)));
        control_RPM_pub.publish(rpm_array);

        // ---- 发布 Odom ----
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.header.stamp    = now_time;
        Eigen::Vector3d pos = sim.getPos();
        Eigen::Vector3d vel = sim.getVel();
        Eigen::Vector3d acc = sim.getAcc();
        Eigen::Vector3d angular_vel = sim.getAngularVelBody();
        Eigen::Vector4d quat = sim.getQuat();
        Eigen::Matrix3d R_body2world = sim.getR();
        Eigen::Vector3d angular_vel_world = R_body2world * angular_vel;

        odom.pose.pose.position.x    = pos(0);
        odom.pose.pose.position.y    = pos(1);
        odom.pose.pose.position.z    = pos(2);
        odom.pose.pose.orientation.w = quat(0);
        odom.pose.pose.orientation.x = quat(1);
        odom.pose.pose.orientation.y = quat(2);
        odom.pose.pose.orientation.z = quat(3);
        odom.twist.twist.linear.x    = vel(0);
        odom.twist.twist.linear.y    = vel(1);
        odom.twist.twist.linear.z    = vel(2);
        odom.twist.twist.angular.x   = angular_vel_world(0);
        odom.twist.twist.angular.y   = angular_vel_world(1);
        odom.twist.twist.angular.z   = angular_vel_world(2);
        odom_pub.publish(odom);

        // ---- 发布 IMU ----
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id    = "/" + quad_name;
        imu_msg.header.stamp       = now_time;
        imu_msg.orientation.w      = quat(0);
        imu_msg.orientation.x      = quat(1);
        imu_msg.orientation.y      = quat(2);
        imu_msg.orientation.z      = quat(3);
        imu_msg.angular_velocity.x = angular_vel(0);
        imu_msg.angular_velocity.y = angular_vel(1);
        imu_msg.angular_velocity.z = angular_vel(2);
        Eigen::Vector3d acc_imu = R_body2world.inverse() * (acc + Eigen::Vector3d(0, 0, -9.8));
        imu_msg.linear_acceleration.x = acc_imu(0);
        imu_msg.linear_acceleration.y = acc_imu(1);
        imu_msg.linear_acceleration.z = acc_imu(2);
        imu_pub.publish(imu_msg);


        {
            geometry_msgs::Point pos_error_tmp;
            pos_error_tmp.x =  pos_des.x() - odom.pose.pose.position.x;
            pos_error_tmp.y =  pos_des.y() - odom.pose.pose.position.y; 
            pos_error_tmp.z =  pos_des.z() - odom.pose.pose.position.z;
            pos_error.publish(pos_error_tmp);

            geometry_msgs::Point vel_error_tmp;
            vel_error_tmp.x =  vel_des.x() - odom.twist.twist.linear.x;
            vel_error_tmp.y =  vel_des.y() - odom.twist.twist.linear.y;
            vel_error_tmp.z =  vel_des.z() - odom.twist.twist.linear.z;
            vel_error.publish(vel_error_tmp);
        }


        rate.sleep();
    }

    return 0;
}