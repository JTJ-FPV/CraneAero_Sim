#pragma once
/**
 * coaxial_x8_allocator.hpp  —  X8 control allocator using CFD model
 *
 */
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include "coaxial_rotor_model.hpp"

class CoaxialX8Allocator
{
public:
    struct Params
    {
        double arm_length = 1.65;
        double min_rpm    = CoaxialRotorModel::rpmMin();
        double max_rpm    = CoaxialRotorModel::rpmMax();
    };

    static constexpr int N_PAIRS = 4;
    // Same yaw sign convention as dynamics
    static constexpr std::array<double, N_PAIRS> PAIR_YAW_SIGN = {+1.0, -1.0, +1.0, -1.0};

private:
    Params p_;
    CoaxialRotorModel aero_model_;

public:
    explicit CoaxialX8Allocator(const Params& p) : p_(p) {}

    /**
     * Allocate 8 motor RPMs from desired wrench.
     *
     * @param thrust_cmd  Total body-z thrust demand [N], positive upward
     * @param tau_cmd     [Mx, My, Mz] moment demand in body frame [N·m]
     * @return            rpm[8]: [u0,l0, u1,l1, u2,l2, u3,l3]
     */
    Eigen::Matrix<double, 8, 1> allocate(double thrust_cmd,
                                          const Eigen::Vector3d& tau_cmd) const
    {
        Eigen::Matrix<double, 8, 1> rpm;
        rpm.setZero();

        // ============================================================
        // Step 1: Allocate per-pair THRUST from total thrust + roll + pitch
        // ============================================================
        //
        // Pair positions (projected arm in X-config):
        //   Pair 0 (front-right):  r = (+a, -a)
        //   Pair 1 (rear-left):    r = (-a, +a)
        //   Pair 2 (front-left):   r = (+a, +a)
        //   Pair 3 (rear-right):   r = (-a, -a)
        //
        // Roll  (Mx) = Σ (-y_i) * T_pair_i
        // Pitch (My) = Σ (+x_i) * T_pair_i    (NOTE: depends on sign convention)
        //
        const double a = p_.arm_length * std::sqrt(2.0) / 2.0;

        // Allocation matrix: [T; Mx; My] = A * [T0; T1; T2; T3]
        Eigen::Matrix<double, 3, 4> A;
        A << 1.0,  1.0,  1.0,  1.0,       // total thrust
             a,   -a,   -a,    a,          // roll  (Mx = Σ -y_i * Ti)
            -a,    a,   -a,    a;          // pitch (My = Σ  x_i * Ti)
        // row signs: pair0 at (+a,-a) → roll contrib = -(-a)*T0 = +a*T0
        //                              → pitch contrib = (+a)*T0 = ... check

        // Actually let me be more careful.
        // motor_pos for pair centers:
        //   pair 0: (a, -a)  → Mx += (-(-a))*T0 = a*T0,   My += (a)*T0
        //   pair 1: (-a, a)  → Mx += (-(a))*T1 = -a*T1,  My += (-a)*T1
        //   pair 2: (a, a)   → Mx += (-(a))*T2 = -a*T2,  My += (a)*T2
        //   pair 3: (-a, -a) → Mx += (-(-a))*T3 = a*T3,   My += (-a)*T3
        //
        // Wait: moment = r × F, F = T*z_hat = (0,0,T)
        // r × (0,0,T) = (y*T, -x*T, 0)
        // So: Mx = Σ y_i * Ti,  My = Σ (-x_i) * Ti

        // Pair centers:
        //   pair 0: x=+a, y=-a → Mx contribution = -a*T0, My contribution = -a*T0
        //   pair 1: x=-a, y=+a → Mx contribution = +a*T1, My contribution = +a*T1
        //   pair 2: x=+a, y=+a → Mx contribution = +a*T2, My contribution = -a*T2
        //   pair 3: x=-a, y=-a → Mx contribution = -a*T3, My contribution = +a*T3

        A << 1.0,  1.0,  1.0,  1.0,       // Σ Ti = thrust_cmd
            -a,    a,    a,   -a,          // Mx = Σ y_i * Ti
            -a,    a,   -a,    a;          // My = Σ (-x_i) * Ti

        Eigen::Vector3d b;
        b << thrust_cmd, tau_cmd(0), tau_cmd(1);

        // Least-squares solve (3 equations, 4 unknowns → 1 DOF null space)
        Eigen::Vector4d T_pair =
            A.completeOrthogonalDecomposition().solve(b);

        // Enforce non-negative thrust per pair
        for (int i = 0; i < 4; ++i)
            T_pair(i) = std::max(0.0, T_pair(i));

        // ============================================================
        // Step 2: Allocate per-pair YAW TORQUE from Mz demand
        // ============================================================
        //
        // Total yaw moment: Mz = Σ PAIR_YAW_SIGN[p] * Q_total_p
        // Distribute equally:  Q_total_p = PAIR_YAW_SIGN[p] * Mz / N_PAIRS
        //
        // Verification: Mz_achieved = Σ sign_p * (sign_p * Mz/4) = 4 * Mz/4 = Mz ✓

        const double Mz = tau_cmd(2);

        // ============================================================
        // Step 3: Solve each pair (T, Q) → (RPM_u, RPM_l) via RBF invert
        // ============================================================
        for (int p = 0; p < N_PAIRS; ++p) {
            const double T_des = T_pair(p);
            const double Q_des = PAIR_YAW_SIGN[p] * Mz / static_cast<double>(N_PAIRS);

            auto inv = aero_model_.invert(T_des, Q_des);

            double wu = std::clamp(inv.omega_u, p_.min_rpm, p_.max_rpm);
            double wl = std::clamp(inv.omega_l, p_.min_rpm, p_.max_rpm);

            rpm(2 * p)     = wu;
            rpm(2 * p + 1) = wl;
        }

        return rpm;
    }

    /**
     * Convenience: compute hover RPMs for a given vehicle weight.
     *
     * @param weight_N  Total vehicle weight [N]  (= mass * g)
     * @return          rpm[8] for symmetric hover
     */
    Eigen::Matrix<double, 8, 1> hoverRPM(double weight_N) const
    {
        const double T_per_pair = weight_N / static_cast<double>(N_PAIRS);

        // Q ≈ 0 for yaw-balanced hover (symmetric RPM)
        // Actually hover Q_total ≈ 4-6 Nm per pair, but contributions cancel
        // due to alternating yaw signs. So we can request Q=0 baseline.
        // The natural Q_total at symmetric RPM is what we get.
        auto inv = aero_model_.invert(T_per_pair, 0.0);

        // In hover, Q_total is small but nonzero (net Q_total ≈ 5-8 Nm positive).
        // Pairs with opposite yaw signs cancel: Mz_net = 0.
        // We use the same RPMs for all pairs — yaw cancels by construction.

        Eigen::Matrix<double, 8, 1> rpm;
        for (int p = 0; p < N_PAIRS; ++p) {
            rpm(2 * p)     = inv.omega_u;
            rpm(2 * p + 1) = inv.omega_l;
        }
        return rpm;
    }
};