#pragma once
/**
 * coaxial_x8_dynamics.hpp  —  X8 coaxial multirotor dynamics
 */
#include <array>
#include <algorithm>
#include <cmath>
#include "dynamics.hpp"
#include "coaxial_rotor_model.hpp"
#include "fuselage_aero_model.hpp"

class coaxial_x8_dynamics : public dynamics
{
public:
    static constexpr int N_ROTORS = 8;
    static constexpr int N_PAIRS  = 4;

    // Pair structure: pairs_[p] = {upper_rotor_idx, lower_rotor_idx}
    static constexpr std::array<std::pair<int,int>, N_PAIRS> PAIRS = {
        {{0,1}, {2,3}, {4,5}, {6,7}}
    };

    // Yaw sign per pair: +1 if pair's CFD Q convention matches airframe yaw,
    // -1 if opposite spin direction.
    // With spin_ = {+1,-1, -1,+1, +1,-1, -1,+1}, pairs 0,2 match CFD, pairs 1,3 are flipped.
    static constexpr std::array<double, N_PAIRS> PAIR_YAW_SIGN = {+1.0, -1.0, +1.0, -1.0};

private:
    CoaxialRotorModel aero_model_;   // CFD-based RBF (built at construction)
    FuselageAeroModel fuselage_aero_;      //  CFD-based RBF fuselage
    
    Eigen::Matrix<double, N_ROTORS, 1> motor_rpm_cmd_ = Eigen::Matrix<double, N_ROTORS, 1>::Zero();
    Eigen::Matrix<double, N_ROTORS, 1> motor_rpm_     = Eigen::Matrix<double, N_ROTORS, 1>::Zero();

    Eigen::Matrix<double, N_ROTORS, 3> motor_pos_;
    Eigen::Matrix<double, N_ROTORS, 3> motor_axis_;

    double arm_length_    = 1.65;
    double z_sep_         = 0.40;    // vertical separation between upper and lower rotor
    double min_rpm_       = CoaxialRotorModel::rpmMin();  // [BUG-2] match CFD range
    double max_rpm_       = CoaxialRotorModel::rpmMax();
    double motor_tau_     = 0.03;    // first-order motor time constant [s]
    double g_             = 9.81;

    Eigen::Vector3d drag_linear_body_  = Eigen::Vector3d(0.08, 0.08, 0.12);
    Eigen::Vector3d drag_angular_body_ = Eigen::Vector3d(0.0008, 0.0008, 0.0012);

public:
    using dynamics::dynamics;

    void init(const Eigen::Vector3d& p_w, const Eigen::Vector4d& quat_wxyz)
    {
        initialize(p_w, quat_wxyz);

        // Motor thrust axes: all point +z in body frame
        motor_axis_.setZero();
        for (int i = 0; i < N_ROTORS; ++i) motor_axis_.row(i) << 0.0, 0.0, 1.0;

        const double a  = arm_length_ * std::sqrt(2.0) / 2.0;
        const double zu = +0.5 * z_sep_;
        const double zl = -0.5 * z_sep_;

        // X8 layout: 4 arms, each arm = upper (even idx) + lower (odd idx)
        // Body frame: x forward, y left, z up
        //   Pair 0  front-right
        motor_pos_.row(0) <<  a, -a, zu;
        motor_pos_.row(1) <<  a, -a, zl;
        //   Pair 1  rear-left
        motor_pos_.row(2) << -a,  a, zu;
        motor_pos_.row(3) << -a,  a, zl;
        //   Pair 2  front-left
        motor_pos_.row(4) <<  a,  a, zu;
        motor_pos_.row(5) <<  a,  a, zl;
        //   Pair 3  rear-right
        motor_pos_.row(6) << -a, -a, zu;
        motor_pos_.row(7) << -a, -a, zl;

        motor_rpm_.setZero();
        motor_rpm_cmd_.setZero();
    }

    // ---- Configuration ----
    void setGeometry(double arm_length, double z_sep) {
        arm_length_ = arm_length;
        z_sep_ = z_sep;
    }
    void setLimits(double min_rpm, double max_rpm) {
        min_rpm_ = min_rpm;
        max_rpm_ = max_rpm;
    }
    void setMotorTimeConstant(double tau) { motor_tau_ = tau; }
    void setDragCoeffs(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
        drag_linear_body_ = linear;
        drag_angular_body_ = angular;
    }

    // ---- Motor commands ----
    void setRPMCmd(const Eigen::Matrix<double, N_ROTORS, 1>& cmd) {
        motor_rpm_cmd_ = cmd;
        for (int i = 0; i < N_ROTORS; ++i)
            motor_rpm_cmd_(i) = std::clamp(motor_rpm_cmd_(i), min_rpm_, max_rpm_);
    }

    Eigen::Matrix<double, N_ROTORS, 1> getRPM() const { return motor_rpm_; }

    // ---- Access to aero model (for allocator) ----
    const CoaxialRotorModel& aeroModel() const { return aero_model_; }

    // ---- Main simulation step ----
    void step_forward(double dt)
    {
        if (dt <= 1e-6) return;

        updateMotorDynamics(dt);

        const Eigen::Vector3d z_b(0.0, 0.0, 1.0);

        Eigen::Vector3d thrust_b = Eigen::Vector3d::Zero();
        Eigen::Vector3d moment_b = Eigen::Vector3d::Zero();

        // [BUG-1 FIX] Compute per-PAIR aerodynamics from CFD model
        for (int p = 0; p < N_PAIRS; ++p) {
            const int iu = PAIRS[p].first;
            const int il = PAIRS[p].second;

            const double rpm_u = motor_rpm_(iu);
            const double rpm_l = motor_rpm_(il);

            // Query CFD-based RBF model for this pair
            auto aero = aero_model_.query(rpm_u, rpm_l);

            // Thrust forces along body +z
            const Eigen::Vector3d F_upper = aero.T_u * z_b;
            const Eigen::Vector3d F_lower = aero.T_l * z_b;
            thrust_b += F_upper + F_lower;

            // Moments from thrust × position (roll/pitch)
            const Eigen::Vector3d r_u = motor_pos_.row(iu).transpose();
            const Eigen::Vector3d r_l = motor_pos_.row(il).transpose();
            moment_b += r_u.cross(F_upper) + r_l.cross(F_lower);

            // [BUG-3 FIX] Yaw reaction torque — CFD Q is already signed.
            // Apply pair_yaw_sign to account for alternating spin directions.
            // Pairs 0,2 match CFD convention; pairs 1,3 are flipped.
            const double ys = PAIR_YAW_SIGN[p];
            moment_b += ys * aero.Q_u * z_b;   // Q_u > 0 from CFD
            moment_b += ys * aero.Q_l * z_b;   // Q_l < 0 from CFD
        }


        const Eigen::Vector3d vel_b = R_bw_.transpose() * vel_w_;

        const double v_x = vel_b.x();
        const double v_z = vel_b.z();

        const double V = v_x;

        const double AoA_rad = (std::abs(v_x) > 1e-3)
                            ? std::atan2(-v_z, std::abs(v_x))
                            : 0.0;

        const double AoA_deg = AoA_rad * 180.0 / M_PI;

        const double V_q = std::clamp(
            V,
            FuselageAeroModel::V_MIN - 5.0,
            FuselageAeroModel::V_MAX + 5.0
        );

        const double AoA_q = std::clamp(
            AoA_deg,
            FuselageAeroModel::AOA_MIN - 5.0,
            FuselageAeroModel::AOA_MAX + 5.0
        );

        const auto fa = fuselage_aero_.query(V_q, AoA_q);

        Eigen::Vector3d fuselage_force_b  = fa.force;
        Eigen::Vector3d fuselage_torque_b = fa.torque;
    

        // Gravity in body frame
        const Eigen::Vector3d gravity_b =
            mass_ * R_bw_.transpose() * Eigen::Vector3d(0.0, 0.0, -g_);

        // Translational drag (body frame)
        const Eigen::Vector3d drag_b = -drag_linear_body_.cwiseProduct(vel_b);

        // Angular damping (body frame)
        const Eigen::Vector3d aero_damp =
            -drag_angular_body_.cwiseProduct(omega_b_);

        // Assemble
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