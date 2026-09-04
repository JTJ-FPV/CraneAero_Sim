#pragma once
/**
 * coaxial_x8_allocator_qp.hpp  —  Eigen-only X8 control allocator
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
 *     (1) A small active-set solver implemented with Eigen finds feasible
 *         (T_pair, Q_pair) respecting box constraints
 *     (2) For each pair: CoaxialRotorModel::invert(T_pair, Q_pair) → (RPM_u, RPM_l)
 *
 *
 */
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
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

        // Active-set solver tuning (KKT stopping tolerances)
        double eps_abs     = 1e-8;
        double eps_rel     = 1e-8;
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

        // ------------- Solve with the Eigen active-set method ----------
        Eigen::Matrix<double, 8, 1> x_opt;
        bool qp_ok = solveBoundedQP(P, q, lb, ub, x_opt, out.qp_iters);

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

    /**
     * Solve the strictly convex box-constrained QP
     *
     *     min 0.5 x' P x + q' x,   lb <= x <= ub
     *
     * with a primal active-set method.  The problem has only eight variables,
     * so rebuilding and factorising the free-variable Hessian is inexpensive
     * and avoids any external optimisation dependency.  At convergence the
     * free-variable gradient is zero and the active-bound multipliers satisfy
     * the KKT sign conditions.
     */
    bool solveBoundedQP(const Eigen::Matrix<double, 8, 8>& P_dense,
                        const Eigen::Matrix<double, 8, 1>& q_vec,
                        const Eigen::Matrix<double, 8, 1>& lb,
                        const Eigen::Matrix<double, 8, 1>& ub,
                        Eigen::Matrix<double, 8, 1>& x_out,
                        int& n_iters) const
    {
        using Mat8 = Eigen::Matrix<double, N_VARS, N_VARS>;
        using Vec8 = Eigen::Matrix<double, N_VARS, 1>;

        enum BoundState : int { LOWER = -1, FREE = 0, UPPER = 1 };

        n_iters = 0;
        x_out.setZero();

        if (!P_dense.allFinite() || !q_vec.allFinite() ||
            !lb.allFinite() || !ub.allFinite()) {
            return false;
        }
        for (int i = 0; i < N_VARS; ++i) {
            if (lb(i) > ub(i)) {
                return false;
            }
        }

        // Protect against insignificant asymmetry introduced by round-off.
        const Mat8 P = 0.5 * (P_dense + P_dense.transpose());
        Eigen::LDLT<Mat8> full_ldlt(P);
        if (full_ldlt.info() != Eigen::Success || !full_ldlt.isPositive()) {
            return false;
        }

        Vec8 x = full_ldlt.solve(-q_vec);
        if (full_ldlt.info() != Eigen::Success || !x.allFinite()) {
            return false;
        }

        std::array<int, N_VARS> state{};
        state.fill(FREE);
        for (int i = 0; i < N_VARS; ++i) {
            if (x(i) <= lb(i)) {
                x(i) = lb(i);
                state[i] = LOWER;
            } else if (x(i) >= ub(i)) {
                x(i) = ub(i);
                state[i] = UPPER;
            }
        }

        const double bound_scale = std::max(
            1.0, std::max(lb.cwiseAbs().maxCoeff(), ub.cwiseAbs().maxCoeff()));
        const double primal_tol = std::max(1e-12, p_.eps_abs + p_.eps_rel * bound_scale);
        const int max_iterations = std::max(1, p_.max_iter);

        for (int iter = 0; iter < max_iterations; ++iter) {
            n_iters = iter + 1;

            std::vector<int> free_indices;
            free_indices.reserve(N_VARS);
            for (int i = 0; i < N_VARS; ++i) {
                if (state[i] == FREE) free_indices.push_back(i);
                else if (state[i] == LOWER) x(i) = lb(i);
                else x(i) = ub(i);
            }

            // Minimise over the current free-variable subspace.
            Vec8 candidate = x;
            const int n_free = static_cast<int>(free_indices.size());
            if (n_free > 0) {
                Eigen::MatrixXd P_ff(n_free, n_free);
                Eigen::VectorXd rhs(n_free);
                for (int r = 0; r < n_free; ++r) {
                    const int i = free_indices[r];
                    rhs(r) = -q_vec(i);
                    for (int j = 0; j < N_VARS; ++j) {
                        if (state[j] != FREE) rhs(r) -= P(i, j) * x(j);
                    }
                    for (int c = 0; c < n_free; ++c) {
                        P_ff(r, c) = P(i, free_indices[c]);
                    }
                }

                Eigen::LDLT<Eigen::MatrixXd> free_ldlt(P_ff);
                if (free_ldlt.info() != Eigen::Success || !free_ldlt.isPositive()) {
                    return false;
                }
                const Eigen::VectorXd free_solution = free_ldlt.solve(rhs);
                if (free_ldlt.info() != Eigen::Success || !free_solution.allFinite()) {
                    return false;
                }
                for (int r = 0; r < n_free; ++r) {
                    candidate(free_indices[r]) = free_solution(r);
                }
            }

            // Stay feasible while moving to the subspace minimiser.  If a
            // bound is encountered, add it to the active set and resolve.
            const Vec8 direction = candidate - x;
            double alpha = 1.0;
            int hit_index = -1;
            int hit_state = FREE;
            for (const int i : free_indices) {
                if (candidate(i) < lb(i) - primal_tol && direction(i) < 0.0) {
                    const double a = (lb(i) - x(i)) / direction(i);
                    if (a < alpha) {
                        alpha = std::max(0.0, a);
                        hit_index = i;
                        hit_state = LOWER;
                    }
                } else if (candidate(i) > ub(i) + primal_tol && direction(i) > 0.0) {
                    const double a = (ub(i) - x(i)) / direction(i);
                    if (a < alpha) {
                        alpha = std::max(0.0, a);
                        hit_index = i;
                        hit_state = UPPER;
                    }
                }
            }

            x += std::clamp(alpha, 0.0, 1.0) * direction;
            if (hit_index >= 0) {
                x(hit_index) = (hit_state == LOWER) ? lb(hit_index) : ub(hit_index);
                state[hit_index] = hit_state;
                continue;
            }

            // Snap tiny numerical bound violations before checking KKT.
            for (const int i : free_indices) {
                if (x(i) < lb(i)) {
                    x(i) = lb(i);
                    state[i] = LOWER;
                } else if (x(i) > ub(i)) {
                    x(i) = ub(i);
                    state[i] = UPPER;
                }
            }

            const Vec8 gradient = P * x + q_vec;
            const double dual_scale = std::max(
                1.0, std::max(q_vec.cwiseAbs().maxCoeff(), (P * x).cwiseAbs().maxCoeff()));
            const double dual_tol = std::max(1e-12, p_.eps_abs + p_.eps_rel * dual_scale);

            // At a lower bound grad >= 0; at an upper bound grad <= 0.
            // Release the most strongly violating active variable.
            int release_index = -1;
            double worst_violation = dual_tol;
            for (int i = 0; i < N_VARS; ++i) {
                double violation = 0.0;
                if (state[i] == LOWER) violation = -gradient(i);
                else if (state[i] == UPPER) violation = gradient(i);

                if (violation > worst_violation) {
                    worst_violation = violation;
                    release_index = i;
                }
            }

            if (release_index < 0) {
                x_out = x;
                return x_out.allFinite();
            }
            state[release_index] = FREE;
        }

        if (p_.verbose) {
            std::cerr << "Eigen active-set allocator reached max_iter="
                      << max_iterations << '\n';
        }
        x_out = x;
        return false;
    }

};
