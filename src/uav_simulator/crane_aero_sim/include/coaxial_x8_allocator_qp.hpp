#pragma once
/**
 * coaxial_x8_allocator_qp.hpp  —  X8 control allocator via OSQP
 *
 * Design:
 *   Decision variable:  x = [T_pair(0..3);  Q_pair(0..3)]  ∈ R⁸
 *       T_pair[p] : total thrust demand for pair p  [N]
 *       Q_pair[p] : total yaw torque demand for pair p  [N·m]
 *
 *   Wrench mapping  M · x = v_des  where v_des = [F; Mx; My; Mz]:
 *       F  = Σ T_pair[p]
 *       Mx = Σ y_p · T_pair[p]
 *       My = Σ -x_p · T_pair[p]
 *       Mz = Σ yaw_sign[p] · Q_pair[p]
 *
 *   Weighted least-squares cost (priority: thrust > roll/pitch > yaw):
 *       min 0.5 · ||W^{1/2} (M x - v_des)||²  +  0.5 · ε ||x||²
 *     = min 0.5 · xᵀ (MᵀWM + εI) x  -  (MᵀW v_des)ᵀ x
 *
 *     where W = diag(w_T, w_M, w_M, w_Y),   default w_T=10, w_M=5, w_Y=1
 *           ε = 1e-4  (tiny regularizer → unique solution in null space)
 *
 *   Box constraints:
 *       T_pair ∈ [T_min, T_max]   where these come from the CFD data hull
 *       Q_pair ∈ [Q_min, Q_max]
 *
 *   Two-stage solve:
 *     (1) OSQP finds feasible (T_pair, Q_pair) respecting box constraints
 *     (2) For each pair: CoaxialRotorModel::invert(T_pair, Q_pair) → (RPM_u, RPM_l)
 *
 *
 */
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <osqp/osqp.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>
#include <limits>
#include "coaxial_rotor_model.hpp"

class CoaxialX8AllocatorQP
{
public:
    struct Params
    {
        double arm_length = 1.65;
        double min_rpm    = CoaxialRotorModel::rpmMin();
        double max_rpm    = CoaxialRotorModel::rpmMax();

        // Priority weights — thrust > roll/pitch > yaw
        double w_thrust   = 10.0;
        double w_moment   = 5.0;
        double w_yaw      = 1.0;
        double w_reg      = 1e-4;   // regularizer on x itself

        // Solver tuning
        double eps_abs     = 1e-4;
        double eps_rel     = 1e-4;
        int    max_iter    = 200;
        bool   verbose     = false;
    };

    static constexpr int N_PAIRS   = 4;
    static constexpr int N_VARS    = 8;    // 4 T_pair + 4 Q_pair
    static constexpr int N_OUTPUTS = 4;    // F, Mx, My, Mz

    static constexpr std::array<double, N_PAIRS> PAIR_YAW_SIGN = {+1.0, -1.0, +1.0, -1.0};

    struct Result
    {
        Eigen::Matrix<double, 8, 1> rpm;          // Output RPMs
        Eigen::Matrix<double, 4, 1> T_pair;       // Solved per-pair thrust
        Eigen::Matrix<double, 4, 1> Q_pair;       // Solved per-pair yaw torque
        Eigen::Vector4d             wrench_ach;   // Actually achieved [F, Mx, My, Mz]
        bool   qp_success = false;
        bool   invert_ok  = true;
        double residual   = 0.0;                  // ||M x - v_des||
        int    qp_iters   = 0;
    };

private:
    Params p_;
    CoaxialRotorModel aero_model_;

    // Precomputed pair-thrust/yaw bounds (derived from CFD data hull)
    double T_min_, T_max_, Q_min_, Q_max_;

    // Precomputed wrench map M (4 outputs × 8 vars) — only depends on geometry
    Eigen::Matrix<double, 4, 8> M_;

public:
    explicit CoaxialX8AllocatorQP(const Params& p) : p_(p)
    {
        buildWrenchMap();
        computePairBounds();
    }

    /**
     * Allocate from desired body-frame wrench.
     *
     * @param thrust_cmd  Total body-z thrust [N]  (positive up)
     * @param tau_cmd     [Mx, My, Mz] in body frame [N·m]
     * @return            Result with rpm[8] and diagnostics
     */
    Result allocate(double thrust_cmd, const Eigen::Vector3d& tau_cmd) const
    {
        Result out;

        // ------------- Build QP ----------------------------------------
        // cost: 0.5 · xᵀ P x + qᵀ x
        // P = MᵀWM + ε·I    (dense but only 8×8)
        // q = -MᵀW·v_des
        Eigen::Vector4d v_des;
        v_des << thrust_cmd, tau_cmd(0), tau_cmd(1), tau_cmd(2);

        Eigen::Vector4d w_diag;
        w_diag << p_.w_thrust, p_.w_moment, p_.w_moment, p_.w_yaw;

        Eigen::Matrix<double, 8, 8> P =
            M_.transpose() * w_diag.asDiagonal() * M_
          + p_.w_reg * Eigen::Matrix<double, 8, 8>::Identity();

        Eigen::Matrix<double, 8, 1> q =
            -M_.transpose() * w_diag.asDiagonal() * v_des;

        // Box constraints:
        // A = I  (8×8),  lb ≤ x ≤ ub
        Eigen::Matrix<double, 8, 1> lb, ub;
        lb.head<4>().setConstant(T_min_);
        ub.head<4>().setConstant(T_max_);
        lb.tail<4>().setConstant(Q_min_);
        ub.tail<4>().setConstant(Q_max_);

        // ------------- Solve via OSQP ----------------------------------
        Eigen::Matrix<double, 8, 1> x_opt;
        bool qp_ok = solveOSQP(P, q, lb, ub, x_opt, out.qp_iters);

        if (!qp_ok) {
            // ---------- Fallback: unconstrained pseudo-inverse --------
            // M⁺ · v_des  then clamp to bounds (non-optimal but recoverable)
            Eigen::Matrix<double, 8, 4> M_pinv =
                M_.transpose() * (M_ * M_.transpose()).inverse();
            x_opt = M_pinv * v_des;
            for (int i = 0; i < 4; ++i)
                x_opt(i) = std::clamp(x_opt(i), T_min_, T_max_);
            for (int i = 4; i < 8; ++i)
                x_opt(i) = std::clamp(x_opt(i), Q_min_, Q_max_);
            out.qp_success = false;
        } else {
            out.qp_success = true;
        }

        out.T_pair = x_opt.head<4>();
        out.Q_pair = x_opt.tail<4>();
        out.residual = (M_ * x_opt - v_des).norm();

        // ------------- Invert each pair → RPMs -------------------------
        out.invert_ok = true;
        for (int p = 0; p < N_PAIRS; ++p) {
            auto inv = aero_model_.invert(out.T_pair(p), out.Q_pair(p));
            out.rpm(2 * p)     = std::clamp(inv.omega_u, p_.min_rpm, p_.max_rpm);
            out.rpm(2 * p + 1) = std::clamp(inv.omega_l, p_.min_rpm, p_.max_rpm);
            if (!inv.success || inv.objective > 1e-2) out.invert_ok = false;
        }

        // ------------- Compute achieved wrench -------------------------
        out.wrench_ach.setZero();
        for (int p = 0; p < N_PAIRS; ++p) {
            auto aero = aero_model_.query(out.rpm(2 * p), out.rpm(2 * p + 1));
            const double a = p_.arm_length * std::sqrt(2.0) / 2.0;
            const double x_pos[] = {+a, -a, +a, -a};
            const double y_pos[] = {-a, +a, +a, -a};

            out.wrench_ach(0) += aero.T_total;                        // F
            out.wrench_ach(1) += y_pos[p] * aero.T_total;             // Mx
            out.wrench_ach(2) += -x_pos[p] * aero.T_total;            // My
            out.wrench_ach(3) += PAIR_YAW_SIGN[p] * aero.Q_total;     // Mz
        }

        return out;
    }

    // -------- Introspection / debug --------
    double getTmin() const { return T_min_; }
    double getTmax() const { return T_max_; }
    double getQmin() const { return Q_min_; }
    double getQmax() const { return Q_max_; }

private:
    // ================================================================
    void buildWrenchMap()
    {
        const double a = p_.arm_length * std::sqrt(2.0) / 2.0;

        // Pair centers (same convention as dynamics):
        //   0 front-right (+a, -a)
        //   1 rear-left   (-a, +a)
        //   2 front-left  (+a, +a)
        //   3 rear-right  (-a, -a)
        const double x_pos[] = {+a, -a, +a, -a};
        const double y_pos[] = {-a, +a, +a, -a};

        M_.setZero();
        // Row 0: F = Σ T_pair[p]
        for (int p = 0; p < 4; ++p) M_(0, p) = 1.0;
        // Row 1: Mx = Σ y_p · T_pair[p]
        for (int p = 0; p < 4; ++p) M_(1, p) = y_pos[p];
        // Row 2: My = Σ -x_p · T_pair[p]
        for (int p = 0; p < 4; ++p) M_(2, p) = -x_pos[p];
        // Row 3: Mz = Σ yaw_sign[p] · Q_pair[p]
        for (int p = 0; p < 4; ++p) M_(3, 4 + p) = PAIR_YAW_SIGN[p];
    }

    // Compute feasible per-pair (T, Q) bounds by sampling CFD data corners
    void computePairBounds()
    {
        const double rmn = p_.min_rpm;
        const double rmx = p_.max_rpm;

        // Thrust: min at (rmn, rmn), max at (rmx, rmx)
        T_min_ = aero_model_.query(rmn, rmn).T_total;
        T_max_ = aero_model_.query(rmx, rmx).T_total;

        // Torque span — Q > 0 when upper spins faster, Q < 0 when lower faster
        // Max |Q| at corners (rmx, rmn) and (rmn, rmx)
        double Q_posmax = aero_model_.query(rmx, rmn).Q_total;
        double Q_negmax = aero_model_.query(rmn, rmx).Q_total;
        Q_min_ = std::min(Q_posmax, Q_negmax);
        Q_max_ = std::max(Q_posmax, Q_negmax);
    }

    // void computePairBounds()
    // {
    //     T_min_ = aero_model_.query(0.0, 0.0).T_total;
    //     T_max_ = aero_model_.query(p_.max_rpm, p_.max_rpm).T_total;

    //     Q_min_ =  std::numeric_limits<double>::infinity();
    //     Q_max_ = -std::numeric_limits<double>::infinity();

    //     for (int i = 0; i < coaxial_data::N_PTS; ++i) {
    //         const double rpm_u = coaxial_data::DATA[i][0];
    //         const double rpm_l = coaxial_data::DATA[i][1];

    //         // Do not use single-propeller corner cases for coaxial yaw authority.
    //         if (rpm_u <= 1.0 || rpm_l <= 1.0) {
    //             continue;
    //         }

    //         const double Q = coaxial_data::DATA[i][4] + coaxial_data::DATA[i][5];
    //         Q_min_ = std::min(Q_min_, Q);
    //         Q_max_ = std::max(Q_max_, Q);
    //     }

    //     if (!std::isfinite(Q_min_) || !std::isfinite(Q_max_)) {
    //         Q_min_ = -20.0;
    //         Q_max_ =  20.0;
    //     }

    //     // Safety margin: do not ask inverse model to sit on the data boundary.
    //     Q_min_ *= 0.8;
    //     Q_max_ *= 0.8;
    // }

    bool solveOSQP(const Eigen::Matrix<double, 8, 8>& P_dense,
                const Eigen::Matrix<double, 8, 1>& q_vec,
                const Eigen::Matrix<double, 8, 1>& lb,
                const Eigen::Matrix<double, 8, 1>& ub,
                Eigen::Matrix<double, 8, 1>& x_out,
                int& n_iters) const
    {
        using SpMat   = Eigen::SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt>;
        using Triplet = Eigen::Triplet<OSQPFloat, OSQPInt>;

        n_iters = 0;
        x_out.setZero();

        // ---------- Build P (upper triangular only) ----------
        SpMat P(8, 8);
        std::vector<Triplet> P_trips;
        P_trips.reserve(36);

        for (OSQPInt j = 0; j < 8; ++j) {
            for (OSQPInt i = 0; i <= j; ++i) {
                const double v = P_dense(i, j);
                if (std::abs(v) > 1e-14) {
                    P_trips.emplace_back(i, j, static_cast<OSQPFloat>(v));
                }
            }
        }

        P.setFromTriplets(P_trips.begin(), P_trips.end());
        P.makeCompressed();

        // ---------- Wrap P into OSQP CSC ----------
        OSQPCscMatrix P_csc{};
        OSQPCscMatrix_set_data(&P_csc,
                            8, 8,
                            static_cast<OSQPInt>(P.nonZeros()),
                            P.valuePtr(),
                            P.innerIndexPtr(),
                            P.outerIndexPtr());

        // ---------- A = I ----------
        OSQPCscMatrix* A_csc = OSQPCscMatrix_identity(8);
        if (!A_csc) return false;

        // ---------- q / l / u ----------
        std::array<OSQPFloat, 8> q_buf{}, l_buf{}, u_buf{};
        for (int i = 0; i < 8; ++i) {
            q_buf[i] = static_cast<OSQPFloat>(q_vec(i));
            l_buf[i] = static_cast<OSQPFloat>(lb(i));
            u_buf[i] = static_cast<OSQPFloat>(ub(i));
        }

        // ---------- settings ----------
        OSQPSettings* settings = OSQPSettings_new();
        if (!settings) {
            OSQPCscMatrix_free(A_csc);
            return false;
        }

        osqp_set_default_settings(settings);
        settings->verbose   = p_.verbose ? 1 : 0;
        settings->eps_abs   = static_cast<OSQPFloat>(p_.eps_abs);
        settings->eps_rel   = static_cast<OSQPFloat>(p_.eps_rel);
        settings->max_iter  = static_cast<OSQPInt>(p_.max_iter);
        settings->polishing = 0;    // disable polishing for deterministic solve time (no extra factorization)

        // ---------- setup + solve ----------
        OSQPSolver* solver = nullptr;
        OSQPInt ret = osqp_setup(&solver,
                                &P_csc, q_buf.data(),
                                A_csc, l_buf.data(), u_buf.data(),
                                8, 8, settings);

        bool success = false;

        if (ret == 0 && solver) {
            ret = osqp_solve(solver);

            if (ret == 0 &&
                solver->info &&
                (solver->info->status_val == OSQP_SOLVED ||
                solver->info->status_val == OSQP_SOLVED_INACCURATE) &&
                solver->solution && solver->solution->x)
            {
                for (int i = 0; i < 8; ++i)
                    x_out(i) = static_cast<double>(solver->solution->x[i]);

                n_iters = static_cast<int>(solver->info->iter);
                success = true;
            }
        }

        if (solver)   osqp_cleanup(solver);
        if (settings) OSQPSettings_free(settings);
        if (A_csc)    OSQPCscMatrix_free(A_csc);

        return success;
    }

};