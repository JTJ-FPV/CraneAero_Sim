#pragma once
/**
 * visualizer.hpp  —  X8 coaxial RViz visualizer (axis-based mount config)
 *
 * Anti-flicker fixes (cumulative):
 *   1. Single MarkerArray on a single topic (atomic update).
 *   2. lifetime = 0, stamp = ros::Time::now(), frame_locked = true.
 *   3. trajectoryAppend NO LONGER publishes inline. Path is published from
 *      a separate low-rate timer to avoid choking the single-threaded
 *      spinner on a 4 MB Path message every odom tick.
 *   4. History capped at 4000 poses (still > 1 minute at 50 Hz odom).
 *   5. Shared state guarded by a mutex; safe under MultiThreaded /
 *      AsyncSpinner. Defensive even under single-threaded spin.
 *   6. Marker publisher queue = 4 (absorb publish bursts).
 *
 * Required RViz-side change:
 *   - Remove old per-propeller / fuselage Marker displays.
 *   - Add ONE MarkerArray display, topic = vehicle_markers.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <Eigen/Eigen>

// ============================================================
//  Configuration structs
// ============================================================
struct CoaxialConfig
{
    std::string propeller_cw_model_path;
    std::string propeller_ccw_model_path;
    std::string fuselage_model_path;
    double scale_x = 1.0;
    double scale_y = 1.0;
    double scale_z = 1.0;

    std::array<Eigen::Vector3d, 8> propeller_origin;
    std::array<Eigen::Vector3d, 8> propeller_axis;
    std::array<bool, 8>            propeller_is_cw;
    Eigen::Vector3d                fuselage_rpy_deg;
};

struct Vis_Param
{
    CoaxialConfig coaxial_;
    std::string   OriginFrame_;
    std::string   VehicleFrame_;
    bool          useFPV = false;
    double        Rate          = 50.0;
    double        TrajPubRate   = 5.0;     // <-- decoupled, slow
    int           TrajMaxPoses  = 4000;    // <-- hard cap

    double rpm_to_rad_scale = 0.01;
    double max_display_rate = 100.0;
    double min_spin_rate    = 2.0;
};

// ============================================================
//  Visualizer
// ============================================================
class Visualizer
{
public:
    explicit Visualizer(ros::NodeHandle& nh_) : nh(nh_)
    {
        Init_paramters();
        Init_subscriber();
        Init_publisher();
        Init_markers();

        anim_timer_ = nh.createTimer(ros::Duration(1.0 / config_.Rate),
                                     &Visualizer::animateCallback, this);

        // separate, slow timer for the (potentially big) Path message
        traj_timer_ = nh.createTimer(ros::Duration(1.0 / config_.TrajPubRate),
                                     &Visualizer::trajectoryPublishCallback, this);
    }

private:
    ros::NodeHandle nh;

    ros::Publisher  vehicle_markers_pub_;
    ros::Publisher  history_trajectoryPub_;

    ros::Subscriber vehicle_odom_pos_sub_;
    ros::Subscriber cmd_rpm_sub_;

    ros::Timer FPV;
    ros::Timer anim_timer_;
    ros::Timer traj_timer_;

    visualization_msgs::MarkerArray marker_array_;     // [0]=fuselage, [1..8]=props
    nav_msgs::Path                  history_traj;

    geometry_msgs::PoseStamped vehicle_pose;
    nav_msgs::Odometry         vehicle_odom;

    Vis_Param config_;

    std::array<double, 8> propeller_rpm_       {};
    std::array<double, 8> propeller_angle_     {};
    std::array<int,    8> propeller_spin_dir_  {};
    std::array<Eigen::Matrix3d, 8> propeller_mount_R_;

    ros::Time last_anim_time_;
    bool      odom_received_ = false;

    // ---- thread-safety: protects vehicle_pose / vehicle_odom / propeller_rpm_ ----
    mutable std::mutex state_mtx_;
    // ---- protects history_traj (writer = odom CB, reader = traj_timer) ----
    mutable std::mutex traj_mtx_;

    // ============================================================
    //  Init
    // ============================================================
    void Init_publisher()
    {
        // queue=4 absorbs short publish bursts; latched=false (default)
        vehicle_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
                                   "vehicle_markers", 4);
        history_trajectoryPub_ = nh.advertise<nav_msgs::Path>("history_trajectory", 1);
    }

    void Init_subscriber()
    {
        vehicle_odom_pos_sub_ = nh.subscribe<nav_msgs::Odometry>(
            "odom", 1, &Visualizer::vehicleOdometryPoseCallback, this,
            ros::TransportHints().tcpNoDelay());

        cmd_rpm_sub_ = nh.subscribe<std_msgs::Float32MultiArray>(
            "cmd_rpm", 1, &Visualizer::rpmCallback, this,
            ros::TransportHints().tcpNoDelay());
    }

    void Init_paramters()
    {
        nh.param("useFPV",       config_.useFPV,       false);
        nh.param("OriginFrame",  config_.OriginFrame_, std::string("world"));
        nh.param("VehicleFrame", config_.VehicleFrame_, std::string("base_link"));
        nh.param("Rate",         config_.Rate,         50.0);
        nh.param("TrajPubRate",  config_.TrajPubRate,  5.0);
        nh.param("TrajMaxPoses", config_.TrajMaxPoses, 4000);

        nh.param("fuselage_model_path", config_.coaxial_.fuselage_model_path,
                 std::string("package://crane_aero_sim/meshes/CraneAero/body.stl"));
        nh.param("propeller_cw_model_path", config_.coaxial_.propeller_cw_model_path,
                 std::string("package://crane_aero_sim/meshes/CraneAero/propeller_cw.stl"));
        nh.param("propeller_ccw_model_path", config_.coaxial_.propeller_ccw_model_path,
                 std::string("package://crane_aero_sim/meshes/CraneAero/propeller_ccw.stl"));

        nh.param("fuselage_roll",  config_.coaxial_.fuselage_rpy_deg.x(), 0.0);
        nh.param("fuselage_pitch", config_.coaxial_.fuselage_rpy_deg.y(), 0.0);
        nh.param("fuselage_yaw",   config_.coaxial_.fuselage_rpy_deg.z(), 90.0);

        nh.param("scale_x", config_.coaxial_.scale_x, 0.001);
        nh.param("scale_y", config_.coaxial_.scale_y, 0.001);
        nh.param("scale_z", config_.coaxial_.scale_z, 0.001);

        nh.param("rpm_to_rad_scale", config_.rpm_to_rad_scale, 0.01);
        nh.param("max_display_rate", config_.max_display_rate, 100.0);
        nh.param("min_spin_rate",    config_.min_spin_rate,    2.0);

        loadPropellerMounts();

        config_.coaxial_.propeller_is_cw =
            {false, true, true, false, false, true, true, false};
        propeller_spin_dir_ = {+1, -1, -1, +1, +1, -1, -1, +1};

        if (config_.useFPV) {
            FPV = nh.createTimer(ros::Duration(1.0 / config_.Rate),
                                 &Visualizer::FPVCallback, this);
        }
    }

    void loadPropellerMounts()
    {
        const double a  = 1.65 * std::sqrt(2.0) / 2.0;
        const double zu = +0.05;
        const double zl = -0.05;
        const std::array<std::array<double, 3>, 8> default_pos = {{
            {{+a, -a, zu}}, {{+a, -a, zl}},
            {{-a, +a, zu}}, {{-a, +a, zl}},
            {{+a, +a, zu}}, {{+a, +a, zl}},
            {{-a, -a, zu}}, {{-a, -a, zl}}
        }};

        for (int i = 0; i < 8; ++i) {
            const std::string k_pos  = "propeller" + std::to_string(i) + "_origin";
            const std::string k_axis = "propeller" + std::to_string(i) + "_axis";

            std::vector<double> pos;
            if (nh.getParam(k_pos, pos) && pos.size() == 3) {
                config_.coaxial_.propeller_origin[i] << pos[0], pos[1], pos[2];
            } else {
                config_.coaxial_.propeller_origin[i] << default_pos[i][0],
                                                        default_pos[i][1],
                                                        default_pos[i][2];
                ROS_WARN_STREAM("Param " << k_pos << " not found — using default");
            }

            std::vector<double> ax;
            Eigen::Vector3d axis;
            if (nh.getParam(k_axis, ax) && ax.size() == 3) {
                axis << ax[0], ax[1], ax[2];
                if (axis.norm() < 1e-9) axis = Eigen::Vector3d::UnitZ();
                else                    axis.normalize();
            } else {
                axis = Eigen::Vector3d::UnitZ();
            }
            config_.coaxial_.propeller_axis[i] = axis;

            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d::UnitZ(), axis);
            propeller_mount_R_[i] = q.toRotationMatrix();
        }
    }

    void Init_markers()
    {
        marker_array_.markers.clear();
        marker_array_.markers.reserve(9);

        // ---- index 0: fuselage ----
        {
            visualization_msgs::Marker m;
            m.type                        = visualization_msgs::Marker::MESH_RESOURCE;
            m.action                      = visualization_msgs::Marker::ADD;
            m.mesh_resource               = config_.coaxial_.fuselage_model_path;
            m.mesh_use_embedded_materials = true;   // see note in response if it still flickers
            m.header.frame_id             = config_.OriginFrame_;
            m.ns                          = "fuselage";
            m.id                          = 0;
            m.pose.orientation.w          = 1.0;
            m.scale.x                     = config_.coaxial_.scale_x;
            m.scale.y                     = config_.coaxial_.scale_y;
            m.scale.z                     = config_.coaxial_.scale_z;
            m.frame_locked                = true;
            m.lifetime                    = ros::Duration(0.0);
            // sensible default color so RViz never renders fully transparent
            m.color.r = m.color.g = m.color.b = 1.0f;
            m.color.a = 1.0f;
            marker_array_.markers.push_back(m);
        }

        // ---- index 1..8: propellers ----
        for (int i = 0; i < 8; ++i) {
            visualization_msgs::Marker m;
            m.type                        = visualization_msgs::Marker::MESH_RESOURCE;
            m.action                      = visualization_msgs::Marker::ADD;
            m.mesh_resource               = config_.coaxial_.propeller_is_cw[i]
                                          ? config_.coaxial_.propeller_cw_model_path
                                          : config_.coaxial_.propeller_ccw_model_path;
            m.mesh_use_embedded_materials = true;
            m.header.frame_id             = config_.OriginFrame_;
            m.ns                          = "propeller";
            m.id                          = i;
            m.pose.orientation.w          = 1.0;
            m.scale.x                     = config_.coaxial_.scale_x;
            m.scale.y                     = config_.coaxial_.scale_y;
            m.scale.z                     = config_.coaxial_.scale_z;
            m.frame_locked                = true;
            m.lifetime                    = ros::Duration(0.0);
            m.color.r = m.color.g = m.color.b = 1.0f;
            m.color.a = 1.0f;
            marker_array_.markers.push_back(m);
        }

        history_traj.header.frame_id = config_.OriginFrame_;
        last_anim_time_              = ros::Time::now();
    }

    // ============================================================
    //  Callbacks (writers)
    // ============================================================
    void rpmCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        const int n = std::min(static_cast<int>(msg->data.size()), 8);
        for (int i = 0; i < n; ++i) {
            propeller_rpm_[i] = static_cast<double>(msg->data[i]);
        }
    }

    void vehicleOdometryPoseCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        // 1) update shared state quickly
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            vehicle_odom        = *msg;
            vehicle_pose.header = msg->header;
            vehicle_pose.pose   = msg->pose.pose;
            odom_received_      = true;
        }

        // 2) only APPEND to traj here, do NOT publish (publishing handled
        //    by trajectoryPublishCallback at TrajPubRate Hz)
        {
            std::lock_guard<std::mutex> lk(traj_mtx_);
            geometry_msgs::PoseStamped p;
            p.header = msg->header;
            p.pose   = msg->pose.pose;
            history_traj.poses.push_back(p);

            // hard cap: drop oldest in batch when exceeded
            const int cap = std::max(100, config_.TrajMaxPoses);
            if (static_cast<int>(history_traj.poses.size()) > cap) {
                const int drop = static_cast<int>(history_traj.poses.size()) - cap;
                history_traj.poses.erase(history_traj.poses.begin(),
                                         history_traj.poses.begin() + drop);
            }
        }
    }

    // ============================================================
    //  Callbacks (readers / publishers)
    // ============================================================
    void animateCallback(const ros::TimerEvent& /*e*/)
    {
        // snapshot under lock, then do all heavy work lock-free
        nav_msgs::Odometry odom_snap;
        std::array<double, 8> rpm_snap;
        bool have_odom;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            have_odom = odom_received_;
            odom_snap = vehicle_odom;
            rpm_snap  = propeller_rpm_;
        }
        if (!have_odom) return;

        const ros::Time now = ros::Time::now();
        double dt = (now - last_anim_time_).toSec();
        if (dt <= 0.0 || dt > 0.2) dt = 1.0 / config_.Rate;
        last_anim_time_ = now;

        for (int i = 0; i < 8; ++i) {
            const double omega_disp = scaleRpmForDisplay(rpm_snap[i]);
            propeller_angle_[i] += propeller_spin_dir_[i] * omega_disp * dt;
            const double two_pi = 2.0 * M_PI;
            propeller_angle_[i] = std::fmod(propeller_angle_[i], two_pi);
            if (propeller_angle_[i] < 0) propeller_angle_[i] += two_pi;
        }

        publishAll(now, odom_snap);
    }

    void trajectoryPublishCallback(const ros::TimerEvent& /*e*/)
    {
        nav_msgs::Path snap;
        {
            std::lock_guard<std::mutex> lk(traj_mtx_);
            if (history_traj.poses.empty()) return;
            snap = history_traj;                     // 1 copy, off the writer's hot path
        }
        snap.header.stamp = ros::Time::now();
        history_trajectoryPub_.publish(snap);
    }

    void FPVCallback(const ros::TimerEvent& /*e*/)
    {
        static tf2_ros::TransformBroadcaster br_map_ego;
        nav_msgs::Odometry odom_snap;
        bool have_odom;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            have_odom = odom_received_;
            odom_snap = vehicle_odom;
        }
        if (!have_odom) return;

        geometry_msgs::TransformStamped t;
        t.header.stamp            = ros::Time::now();
        t.header.frame_id         = config_.OriginFrame_;
        t.child_frame_id          = config_.VehicleFrame_;
        t.transform.translation.x = odom_snap.pose.pose.position.x;
        t.transform.translation.y = odom_snap.pose.pose.position.y;
        t.transform.translation.z = odom_snap.pose.pose.position.z;
        t.transform.rotation      = odom_snap.pose.pose.orientation;
        br_map_ego.sendTransform(t);
    }

    void publishAll(const ros::Time& stamp, const nav_msgs::Odometry& odom_snap)
    {
        const Eigen::Quaterniond q_wb(odom_snap.pose.pose.orientation.w,
                                      odom_snap.pose.pose.orientation.x,
                                      odom_snap.pose.pose.orientation.y,
                                      odom_snap.pose.pose.orientation.z);
        const Eigen::Vector3d t_wb(odom_snap.pose.pose.position.x,
                                   odom_snap.pose.pose.position.y,
                                   odom_snap.pose.pose.position.z);
        const Eigen::Matrix3d R_wb = q_wb.normalized().toRotationMatrix();

        const Eigen::Quaterniond q_corr = rpyDegToQuat(config_.coaxial_.fuselage_rpy_deg);
        const Eigen::Matrix3d   R_corr = q_corr.toRotationMatrix();

        // ---- fuselage ----
        {
            Eigen::Quaterniond q_fus = q_wb * q_corr;
            q_fus.normalize();

            auto& m = marker_array_.markers[0];
            m.header.stamp       = stamp;
            m.pose.position.x    = t_wb.x();
            m.pose.position.y    = t_wb.y();
            m.pose.position.z    = t_wb.z();
            m.pose.orientation.w = q_fus.w();
            m.pose.orientation.x = q_fus.x();
            m.pose.orientation.y = q_fus.y();
            m.pose.orientation.z = q_fus.z();
        }

        // ---- propellers ----
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3d& t_mount_raw = config_.coaxial_.propeller_origin[i];
            const Eigen::Matrix3d& R_mount_raw = propeller_mount_R_[i];

            const Eigen::Vector3d t_bm = R_corr * t_mount_raw;
            const Eigen::Matrix3d R_bm = R_corr * R_mount_raw;

            const Eigen::Matrix3d R_spin =
                Eigen::AngleAxisd(propeller_angle_[i], Eigen::Vector3d::UnitZ()).toRotationMatrix();

            const Eigen::Vector3d t_wp = t_wb + R_wb * t_bm;
            const Eigen::Matrix3d R_wp = R_wb * R_bm * R_spin;

            Eigen::Quaterniond q_wp(R_wp);
            q_wp.normalize();

            auto& m = marker_array_.markers[1 + i];
            m.header.stamp       = stamp;
            m.pose.position.x    = t_wp.x();
            m.pose.position.y    = t_wp.y();
            m.pose.position.z    = t_wp.z();
            m.pose.orientation.w = q_wp.w();
            m.pose.orientation.x = q_wp.x();
            m.pose.orientation.y = q_wp.y();
            m.pose.orientation.z = q_wp.z();
        }

        vehicle_markers_pub_.publish(marker_array_);
    }

    // ============================================================
    //  Helpers
    // ============================================================
    double scaleRpmForDisplay(double rpm) const
    {
        const double omega  = std::abs(rpm) * config_.rpm_to_rad_scale;
        const double capped = std::min(omega, config_.max_display_rate);
        if (std::abs(rpm) < 1e-3) return 0.0;
        return std::max(capped, config_.min_spin_rate);
    }

    Eigen::Quaterniond rpyDegToQuat(const Eigen::Vector3d& rpy_deg) const
    {
        const double r = rpy_deg.x() * M_PI / 180.0;
        const double p = rpy_deg.y() * M_PI / 180.0;
        const double y = rpy_deg.z() * M_PI / 180.0;
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
        q.normalize();
        return q;
    }
};