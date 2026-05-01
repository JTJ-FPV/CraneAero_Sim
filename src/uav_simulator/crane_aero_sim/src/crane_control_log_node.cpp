#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <csignal>
#include <atomic>
#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>

#include <Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>

#include "dynamics.hpp"
#include "coaxial_rotor_model.hpp"
#include "coaxial_x8_dynamics.hpp"
#include "coaxial_x8_allocator.hpp"
#include "coaxial_x8_allocator_qp.hpp"
#include "coaxial_x8_controller.hpp"

static double unwrapYaw(double yaw_new) {
    static double prev = 0.0;
    static bool   init = false;
    if (init) {
        while (yaw_new - prev >  M_PI) yaw_new -= 2.0 * M_PI;
        while (yaw_new - prev < -M_PI) yaw_new += 2.0 * M_PI;
    } else {
        init = true;
    }
    prev = yaw_new;
    return yaw_new;
}
// ===================== 控制模式 =====================
enum ControlMode {
    RC_CONTROL       = 0,
    CMD_WAITING      = 1,
    POSITION_CONTROL = 2,
    PLANNING_CONTROL = 3,
};

static const int MODE_TOGGLE_BUTTON = 8;

ControlMode current_control_mode = RC_CONTROL;
bool        snap_setpoint_to_current = false;

// ---- Publishers ----
ros::Publisher control_RPM_pub, odom_pub, imu_pub;
ros::Publisher setpoint_pose_pub;       // 当前激活的 setpoint (PoseStamped)
ros::Publisher setpoint_path_pub;       // 历史 setpoint (Path)
ros::Publisher actual_path_pub;         // 历史实际位置 (Path)

Eigen::Vector3d pos_des, vel_des, acc_des;
geometry_msgs::PoseStamped pose_cmd;
double yaw_des = 0.0;

RcInput rc_input;

// ===================== Logger =====================
//
//  CSV 实时写入磁盘，每行一帧（仿真 200 Hz → 200 行/秒）
//  发现 SIGINT 时 flush + 关闭文件，避免数据丢失。
//
class CsvLogger
{
public:
    bool open(const std::string& path) {
        path_ = path;
        f_.open(path);
        if (!f_) return false;
        f_ << "t,mode,"
              "px_des,py_des,pz_des,"
              "vx_des,vy_des,vz_des,"
              "ax_des,ay_des,az_des,"
              "yaw_des,"
              "px,py,pz,"
              "vx,vy,vz,"
              "qw,qx,qy,qz,"
              "wx,wy,wz,"
              "thrust_cmd,tau_x,tau_y,tau_z,"
              "rpm0,rpm1,rpm2,rpm3,rpm4,rpm5,rpm6,rpm7,"
              "alloc_residual,qp_iter\n";
        return true;
    }

    void writeRow(double t, int mode,
                  const Eigen::Vector3d& p_des, const Eigen::Vector3d& v_des,
                  const Eigen::Vector3d& a_des, double yaw_des,
                  const Eigen::Vector3d& p,     const Eigen::Vector3d& v,
                  const Eigen::Vector4d& q,     const Eigen::Vector3d& w,
                  double thrust, const Eigen::Vector3d& tau,
                  const Eigen::Matrix<double, 8, 1>& rpm,
                  double residual, int qp_iter)
    {
        if (!f_.is_open()) return;
        f_ << std::fixed << std::setprecision(6) << t << ","
           << mode << ","
           << p_des.x() << "," << p_des.y() << "," << p_des.z() << ","
           << v_des.x() << "," << v_des.y() << "," << v_des.z() << ","
           << a_des.x() << "," << a_des.y() << "," << a_des.z() << ","
           << yaw_des << ","
           << p.x() << "," << p.y() << "," << p.z() << ","
           << v.x() << "," << v.y() << "," << v.z() << ","
           << q(0) << "," << q(1) << "," << q(2) << "," << q(3) << ","
           << w.x() << "," << w.y() << "," << w.z() << ","
           << thrust << ","
           << tau.x() << "," << tau.y() << "," << tau.z() << ",";
        for (int i = 0; i < 8; ++i) f_ << rpm(i) << ",";
        f_ << residual << "," << qp_iter << "\n";
    }

    void close() {
        if (f_.is_open()) { f_.flush(); f_.close(); }
    }

    const std::string& path() const { return path_; }
    bool ok() const { return f_.is_open(); }

private:
    std::string   path_;
    std::ofstream f_;
};

static CsvLogger g_logger;
static std::atomic<bool> g_shutdown_requested{false};

void sigintHandler(int /*sig*/) {
    g_shutdown_requested = true;
    g_logger.close();
    ROS_WARN("SIGINT received: log saved -> %s", g_logger.path().c_str());
    ros::shutdown();
}

// 生成时间戳文件名 ~/.ros/crane_log_YYYYMMDD_HHMMSS.csv
std::string makeTimestampedLogPath(const std::string& dir)
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << dir << "/crane_log_"
        << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    return oss.str();
}


// ===================== 回调 =====================
void cmd_callback(const geometry_msgs::PoseStamped& msg)
{
    pose_cmd = msg;
    if (current_control_mode == CMD_WAITING) {
        current_control_mode = POSITION_CONTROL;
        rc_input.active = false;
        ROS_WARN("[Mode] CMD_WAITING -> POSITION_CONTROL  (cmd_pose arrived first)");
    }
}

void planning_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
    if (current_control_mode == RC_CONTROL ||
        current_control_mode == POSITION_CONTROL) {
        return;
    }
    if (current_control_mode == CMD_WAITING) {
        current_control_mode = PLANNING_CONTROL;
        rc_input.active = false;
        ROS_WARN("[Mode] CMD_WAITING -> PLANNING_CONTROL (planning_cmd arrived first)");
    }
    pos_des = Eigen::Vector3d(cmd->position.x,     cmd->position.y,     cmd->position.z);
    vel_des = Eigen::Vector3d(cmd->velocity.x,     cmd->velocity.y,     cmd->velocity.z);
    acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    yaw_des = unwrapYaw(cmd->yaw);

    ROS_INFO_THROTTLE(1.0,
        "Received planning command: pos (%.2f, %.2f, %.2f), vel (%.2f, %.2f, %.2f), acc (%.2f, %.2f, %.2f)",
        pos_des.x(), pos_des.y(), pos_des.z(),
        vel_des.x(), vel_des.y(), vel_des.z(),
        acc_des.x(), acc_des.y(), acc_des.z());
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->axes.size() > 4) {
        rc_input.pitch_stick    =  msg->axes[4];
        rc_input.roll_stick     = -msg->axes[3];
        rc_input.throttle_stick =  msg->axes[1];
        rc_input.yaw_stick      =  msg->axes[0];
    }
    static bool prev_btn = false;
    const bool curr_btn = (msg->buttons.size() > MODE_TOGGLE_BUTTON) &&
                          (msg->buttons[MODE_TOGGLE_BUTTON] == 1);
    if (curr_btn && !prev_btn) {
        if (current_control_mode == RC_CONTROL) {
            current_control_mode = CMD_WAITING;
            rc_input.active = false;
            snap_setpoint_to_current = true;
            ROS_WARN("[Mode] btn%d ↑ : RC_CONTROL -> CMD_WAITING  (waiting for first cmd…)",
                     MODE_TOGGLE_BUTTON);
        } else {
            current_control_mode = RC_CONTROL;
            rc_input.active = true;
            ROS_WARN("[Mode] btn%d ↑ : -> RC_CONTROL", MODE_TOGGLE_BUTTON);
        }
    }
    prev_btn = curr_btn;
    rc_input.active = (current_control_mode == RC_CONTROL);
}


// ===================== Inertia model (lumped central-payload) =====================
static constexpr double R_DISK     = 1.65;
static constexpr double F_BODY     = 0.85;
static constexpr double R_BODY     = 0.30;
static constexpr double H_BODY     = 0.50;
static constexpr double Z_SEP_HALF = 0.20;
static constexpr double SQRT_HALF  = 0.70710678118654752;
static constexpr double A_ARM      = R_DISK * SQRT_HALF;

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
    J(0, 0) = kx * m; J(1, 1) = ky * m; J(2, 2) = kz * m;
    return J;
}

// ===================== Helper: build Path-like history =====================
struct PathBuffer {
    std::deque<geometry_msgs::PoseStamped> poses;
    std::size_t max_size = 6000;            // 30 s @ 200 Hz; protect memory
    void push(const geometry_msgs::PoseStamped& p) {
        poses.push_back(p);
        while (poses.size() > max_size) poses.pop_front();
    }
    nav_msgs::Path toPathMsg(const std::string& frame) const {
        nav_msgs::Path msg;
        msg.header.frame_id = frame;
        msg.header.stamp = ros::Time::now();
        msg.poses.assign(poses.begin(), poses.end());
        return msg;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crane_control_node",
              ros::init_options::NoSigintHandler);   // 自管 SIGINT 以便 flush log
    std::signal(SIGINT, sigintHandler);

    ros::NodeHandle nh("~");
    double init_x, init_y, init_z, mass;
    double simulation_rate;
    std::string quad_name, log_dir;
    bool   enable_csv_log;
    nh.param("mass", mass, 300.0);
    nh.param("init_state_x", init_x, 0.0);
    nh.param("init_state_y", init_y, 0.0);
    nh.param("init_state_z", init_z, 1.0);
    nh.param("simulation_rate", simulation_rate, 200.0);
    nh.param("coaxial_name", quad_name, std::string("coaxial_1"));
    nh.param("enable_csv_log", enable_csv_log, true);
    {
        const char* home = std::getenv("HOME");
        nh.param("log_dir", log_dir,
                 std::string(home ? home : ".") + "/.ros");
    }

    odom_pub        = nh.advertise<nav_msgs::Odometry>("odom", 100);
    imu_pub         = nh.advertise<sensor_msgs::Imu>("imu", 10);
    control_RPM_pub = nh.advertise<std_msgs::Float32MultiArray>("cmd_rpm", 100);

    // 新增 publishers
    setpoint_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_pose", 10);
    setpoint_path_pub = nh.advertise<nav_msgs::Path>("setpoint_path", 1, /*latch=*/true);
    actual_path_pub   = nh.advertise<nav_msgs::Path>("actual_path",   1, /*latch=*/true);

    ros::Subscriber cmd_sub          = nh.subscribe("cmd_pose",     100, &cmd_callback,          ros::TransportHints().tcpNoDelay());
    ros::Subscriber planning_cmd_sub = nh.subscribe("planning_cmd", 100, &planning_cmd_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber joy_sub          = nh.subscribe("/joy",         10,  &joyCallback,           ros::TransportHints().tcpNoDelay());

    pos_des = Eigen::Vector3d(init_x, init_y, init_z);
    vel_des = Eigen::Vector3d::Zero();
    acc_des = Eigen::Vector3d::Zero();

    CoaxialX8Controller ctrl(simulation_rate);
    Eigen::Matrix3d I = inertiaForMass(mass);
    const double g = 9.81;

    CoaxialX8Controller::Params cp;
    cp.mass = mass; cp.inertia = I; cp.g = g;
    ctrl.setParams(cp);

    RcConfig rc_cfg;
    rc_cfg.deadzone     = 0.05;
    rc_cfg.expo         = 0.3;
    rc_cfg.max_vel_xy   = 3.0;
    rc_cfg.max_vel_z    = 2.0;
    rc_cfg.max_yaw_rate = 25.0 * M_PI / 180.0;
    ctrl.setRcConfig(rc_cfg);

    CoaxialX8Controller::Gains cg;
    cg.pos_stable_time = 3.2;
    cg.alt_stable_time = 3.8;
    cg.att_stable_time = 0.9;
    cg.att_damping     = 0.9;
    cg.yaw_rel_kp      = 1.0;
    cg.yaw_rel_kd      = 1.0;
    ctrl.setGains(cg);

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
    if (cos_tilt_min >= 1.0) ROS_FATAL("TWR < 1 — Cannot take off!");
    cl.max_tilt_rad     = std::acos(cos_tilt_min) * 0.5;
    cl.max_acc_z_up     =  1.5;
    cl.max_acc_z_down   = -1.0;
    cl.max_acc_xy       =  1.5;
    cl.max_yaw_rate_rad = 25.0 * M_PI / 180.0;
    ctrl.setLimits(cl);

    coaxial_x8_dynamics sim(mass, I);
    sim.setGeometry(1.65, 0.40);
    sim.init(Eigen::Vector3d(init_x, init_y, init_z),
             Eigen::Vector4d(1, 0, 0, 0));

    // ----------- Open CSV log -----------
    if (enable_csv_log) {
        const std::string log_path = makeTimestampedLogPath(log_dir);
        if (g_logger.open(log_path)) {
            ROS_INFO("CSV log file: %s", log_path.c_str());
        } else {
            ROS_ERROR("Failed to open log file at %s", log_path.c_str());
        }
    }

    PathBuffer setpoint_history;
    PathBuffer actual_history;
    const std::string world_frame = "world";

    ros::Rate rate(simulation_rate);
    ros::Time t_start = ros::Time::now();
    ros::Time last_time = t_start;
    int iter_count = 0;

    while (ros::ok() && !g_shutdown_requested)
    {
        ros::spinOnce();

        // ---- 反馈 ----
        Eigen::Vector4d q = sim.getQuat();
        Eigen::Quaterniond q_wb(q(0), q(1), q(2), q(3));
        ctrl.setFeedback(sim.getPos(), sim.getVel(),
                         q_wb, sim.getAngularVelBody());

        // ---- RC latch reset ----
        static ControlMode last_mode = RC_CONTROL;
        if (last_mode == RC_CONTROL && current_control_mode != RC_CONTROL) {
            ctrl.resetRcLatch();
        }
        last_mode = current_control_mode;

        if (snap_setpoint_to_current) {
            pos_des = sim.getPos();
            vel_des.setZero();
            acc_des.setZero();
            const Eigen::Vector3d xb = q_wb.toRotationMatrix() * Eigen::Vector3d::UnitX();
            yaw_des = unwrapYaw(std::atan2(xb.y(), xb.x()));
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
                ctrl.setSetpoint(pos_des, vel_des, acc_des, yaw_des);
                break;
            case CMD_WAITING:
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

        ros::Time now_time = ros::Time::now();
        double dt = (now_time - last_time).toSec();
        last_time = now_time;
        if (dt <= 0.0 || dt > 0.02) dt = 1.0 / simulation_rate;

        sim.step_forward(dt);

        // ============ 发布 setpoint pose / path ============
        // Note: 在 RC 模式下也发布，setpoint 由控制器内部 latch 保持
        geometry_msgs::PoseStamped sp_msg;
        sp_msg.header.frame_id = world_frame;
        sp_msg.header.stamp    = now_time;
        sp_msg.pose.position.x = pos_des.x();
        sp_msg.pose.position.y = pos_des.y();
        sp_msg.pose.position.z = pos_des.z();
        // 用 yaw_des 构成只含偏航的四元数，便于 RViz 显示朝向
        const double half = 0.5 * yaw_des;
        sp_msg.pose.orientation.w = std::cos(half);
        sp_msg.pose.orientation.x = 0.0;
        sp_msg.pose.orientation.y = 0.0;
        sp_msg.pose.orientation.z = std::sin(half);
        setpoint_pose_pub.publish(sp_msg);
        setpoint_history.push(sp_msg);

        geometry_msgs::PoseStamped act_msg;
        act_msg.header = sp_msg.header;
        act_msg.pose.position.x = sim.getPos().x();
        act_msg.pose.position.y = sim.getPos().y();
        act_msg.pose.position.z = sim.getPos().z();
        act_msg.pose.orientation.w = q(0);
        act_msg.pose.orientation.x = q(1);
        act_msg.pose.orientation.y = q(2);
        act_msg.pose.orientation.z = q(3);
        actual_history.push(act_msg);

        // 路径以 5 Hz 发布（200 Hz 做不必要)
        if ((iter_count % 40) == 0) {
            setpoint_path_pub.publish(setpoint_history.toPathMsg(world_frame));
            actual_path_pub  .publish(actual_history  .toPathMsg(world_frame));
        }

        // ============ 写 CSV 日志 ============
        if (g_logger.ok()) {
            double t = (now_time - t_start).toSec();
            g_logger.writeRow(t, static_cast<int>(current_control_mode),
                              pos_des, vel_des, acc_des, yaw_des,
                              sim.getPos(), sim.getVel(),
                              q, sim.getAngularVelBody(),
                              wrench.thrust_body_z, wrench.torque_body,
                              rpm8.rpm, rpm8.residual, rpm8.qp_iters);
        }

        // ============ 发布 RPM / Odom / IMU （原有功能） ============
        std_msgs::Float32MultiArray rpm_array;
        rpm_array.data.clear();
        for (size_t i = 0; i < 8; ++i)
            rpm_array.data.emplace_back(static_cast<float>(rpm8.rpm(i)));
        control_RPM_pub.publish(rpm_array);

        nav_msgs::Odometry odom;
        odom.header.frame_id = world_frame;
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

        ++iter_count;
        rate.sleep();
    }

    // 正常退出（非 SIGINT）也 flush
    g_logger.close();
    return 0;
}