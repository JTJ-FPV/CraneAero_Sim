#pragma once
#include <Eigen/Eigen>
#include <cmath>

class dynamics
{
protected:
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inertia_inv_;
    double mass_ = 1.0;

    Eigen::Quaterniond q_bw_;         // body -> world
    Eigen::Matrix3d R_bw_;            // body -> world

    Eigen::Vector3d pos_w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_w_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d omega_b_ = Eigen::Vector3d::Zero();  // body angular vel
    Eigen::Vector3d alpha_b_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d force_b_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_b_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_w_ = Eigen::Vector3d::Zero();

public:
    dynamics(double m, const Eigen::Matrix3d& I)
        : inertia_(I), mass_(m)
    {
        inertia_inv_ = inertia_.inverse();
        q_bw_.setIdentity();
        R_bw_ = q_bw_.toRotationMatrix();
    }

    virtual ~dynamics() = default;

    void initialize(const Eigen::Vector3d& p_w, const Eigen::Vector4d& quat_wxyz)
    {
        pos_w_ = p_w;
        vel_w_.setZero();
        acc_w_.setZero();
        omega_b_.setZero();
        alpha_b_.setZero();

        q_bw_ = Eigen::Quaterniond(quat_wxyz(0), quat_wxyz(1), quat_wxyz(2), quat_wxyz(3));
        q_bw_.normalize();
        R_bw_ = q_bw_.toRotationMatrix();

        force_b_.setZero();
        force_w_.setZero();
        torque_b_.setZero();
        torque_w_.setZero();
    }

    virtual void step(double dt)
    {
        if (dt <= 1e-6) return;

        // translational
        acc_w_ = force_w_ / mass_;
        vel_w_ += acc_w_ * dt;
        pos_w_ += vel_w_ * dt;

        // rotational in body frame
        alpha_b_ = inertia_inv_ * (torque_b_ - omega_b_.cross(inertia_ * omega_b_));
        omega_b_ += alpha_b_ * dt;

        // quaternion integration
        Eigen::Quaterniond omega_q(0.0, omega_b_(0), omega_b_(1), omega_b_(2));
        Eigen::Quaterniond qdot = q_bw_ * omega_q;
        q_bw_.coeffs() += 0.5 * dt * qdot.coeffs();
        q_bw_.normalize();
        R_bw_ = q_bw_.toRotationMatrix();
    }

    Eigen::Vector3d getPos() const { return pos_w_; }
    Eigen::Vector3d getVel() const { return vel_w_; }
    Eigen::Vector3d getAcc() const { return acc_w_; }
    Eigen::Vector3d getAngularVelBody() const { return omega_b_; }
    Eigen::Vector3d getAngularAccBody() const { return alpha_b_; }
    Eigen::Matrix3d getR() const { return R_bw_; }

    Eigen::Vector4d getQuat() const
    {
        Eigen::Vector4d q;
        q << q_bw_.w(), q_bw_.x(), q_bw_.y(), q_bw_.z();
        return q;
    }

    Eigen::Vector3d getForceBody() const { return force_b_; }
    Eigen::Vector3d getTorqueBody() const { return torque_b_; }

    void setExternalWrenchBody(const Eigen::Vector3d& force_b, const Eigen::Vector3d& torque_b)
    {
        force_b_ = force_b;
        torque_b_ = torque_b;
        force_w_ = R_bw_ * force_b_;
        torque_w_ = R_bw_ * torque_b_;
    }
};