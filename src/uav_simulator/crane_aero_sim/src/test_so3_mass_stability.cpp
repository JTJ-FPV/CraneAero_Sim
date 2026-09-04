// ============================================================================
//  test_so3_mass_stability.cpp
//
//  Closed-loop SO(3) controller stability sweep for a CraneAero-class
//  coaxial X8 UAV under fixed (T = 300 K) rotor aerodynamic data.
//
//  Two questions are answered:
//    Q1. How does increased payload mass (with proportional inertia growth)
//        degrade tracking performance, and which failure mode appears first?
//    Q2. At what mass does the controller cease to keep state bounded?
//
//  Method
//  ------
//  For each mass value m_i, build a fresh
//        dynamics + controller + Eigen QP-allocator stack
//  with mass set to m_i and inertia set to  J(m_i) = m_i · diag(Jx, Jy, Jz),
//  then run a 25-second closed-loop simulation:
//
//      Phase A  (0  – 5 s)   hover hold at (0, 0, 5)  → altitude collapse?
//      Phase B  (5  – 15 s)  step to     (2, 0, 5)    → horizontal authority?
//      Phase C  (15 – 25 s)  return to   (0, 0, 5)    → bounded recovery?
//
//  Metrics per trial:
//      max_pos_err, rms_pos_err (over Phase B+C),
//      peak_tilt_deg, peak_rpm, final_z, divergence flag, hover_failed flag.
//
//  Verdict tiers:
//      OK         RMS ≤ 0.5 m, peak_tilt < 25°, no persistent saturation
//      DEGRADED   RMS ≤ 2.0 m, peak_tilt < 45°, occasional saturation
//      POOR       RMS > 2.0 m, persistent saturation
//      UNSTABLE   peak_tilt > 60° or altitude collapse > 2 m
//      DIVERGED   non-finite state or |pos| > 1 km
//
//  Pre-test theoretical bounds (T = 300 K, RBF-cubic surrogate):
//      Per-pair max thrust @ (2245, 2245)            ≈ 1333.6 N
//      Total body-z thrust ceiling   F_max_aero      ≈ 5334   N
//      Pure hover ceiling            m·g ≤ F_max     ≈ 543.7  kg
//      With 15 % attitude reserve    0.85·F_max/g    ≈ 462    kg
//      30° tilt-hover ceiling        F_max·cos30/g   ≈ 471    kg
//
//  Torque-limit scaling policy
//  ---------------------------
//  The default Limits::max_torque_xy_Nm = 20 N·m is appropriate only for the
//  1.9 kg baseline frame.  As mass grows (and inertia with it), this hard-
//  coded limit cuts ever deeper into closed-loop performance, masking the
//  *real* aerodynamic ceiling we actually want to identify.  We therefore
//  scale the torque budget with inertia to keep the controller's intended
//  angular-acceleration capability fixed, then cap at half of the rotor
//  envelope so the inverse model never sits exactly on the data hull:
//
//      τ_max_xy = clamp(α_des × J_xx, 0, 0.5 × τ_aero_xy_max)
//      τ_max_z  = clamp(α_des × J_zz, 0, 0.5 × τ_aero_z_max)
//
//      α_des              = 20 rad/s²    (default's implicit α_max @ J=I)
//      τ_aero_xy_max      ≈ 3120 N·m     (4 × T_pair_max × arm/√2)
//      τ_aero_z_max       ≈  352 N·m     (4 × Q_pair_max with alternating yaw)
//
//  With this policy the mass-failure threshold in the sweep is dominated by
//  *thrust* authority — the true aerodynamic ceiling — rather than by an
//  artificial torque hard-cap that does not correspond to any real
//  rotor limit.
//
//  Build (ROS workspace, alongside the rest of the code base):
//      g++ -std=c++17 -O2 \
//          -I${EIGEN_INC} -I${ROS_INC} \
//          test_so3_mass_stability.cpp \
//          -L${ROS_LIB} -lrosconsole -lroscpp -lrostime \
//          -o test_so3_mass_stability
//
//  Run:
//      ./test_so3_mass_stability                  # writes so3_mass_sweep.csv
//      ./test_so3_mass_stability --decoupled      # also runs inertia-decoupled
//                                                   sub-test at fixed mass
//
//  NOTE on ROS dependency
//  ----------------------
//  CoaxialX8Controller transitively includes <ros/ros.h> and uses
//  ROS_INFO_THROTTLE in updateFromRC() (which we never call here).
//  The macro is harmless without ros::init, but we still call ros::init at
//  the top so that it can be used as a ROS node binary if desired.
// ============================================================================

#include <ros/ros.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "coaxial_x8_controller.hpp"
#include "coaxial_x8_allocator_qp.hpp"
#include "coaxial_x8_dynamics.hpp"

// ----------------------------------------------------------------------------
//  Per-kg moment-of-inertia coefficients (kg·m² / kg).
//
//  Lumped central-payload model — appropriate for CraneAero-class cargo
//  UAVs, where the bulk of the mass (battery + payload + airframe core)
//  is centrally mounted and only motors + ESCs + arms live at the rotor
//  radius:
//
//      • Fraction F_BODY of total mass is concentrated in a central
//        cylindrical body, modelled as a solid cylinder of radius
//        R_BODY and height H_BODY (battery + cargo + frame core).
//      • Fraction (1 − F_BODY) is split among the 8 rotor pods sitting
//        at the four corners (±a, ±a, ±z_sep/2), with a = R_DISK / √2.
//
//  Per-kg coefficients (per unit total mass m):
//
//      KJX = KJY = (1−F)·(a² + z_sep_half²)
//                + F  ·(3 r_body² + h_body²)/12
//
//      KJZ       = (1−F)·(2·a²)
//                + F  · r_body²/2
//
//  With defaults below (F_BODY = 0.85):
//      KJX = KJY ≈ 0.247,   KJZ ≈ 0.447     [kg·m²/kg]
//
//  For comparison:
//      uniform-disk model (R = 1.65)        →  KJX ≈ 0.681,  KJZ ≈ 1.361
//      previous point-mass coefficients     →  KJX ≈ 0.300,  KJZ ≈ 0.550
//      central-payload (F_BODY = 0.85)      →  KJX ≈ 0.247,  KJZ ≈ 0.447   ← here
//      central-payload (F_BODY = 0.95)      →  KJX ≈ 0.105,  KJZ ≈ 0.179
//
//  The central-payload model gives ~37 % of the disk-model inertia, which
//  shifts the τ-cap onset (1500 N·m / J = α_des = 20 rad/s²) from m ≈ 110 kg
//  back to m ≈ 305 kg, restoring the controller's intended attitude
//  bandwidth across a much wider operating range.
//
//  Edit F_BODY in [0.5, 0.95] to model lighter (closer to disk) or heavier
//  (closer to point-mass at centre) cargo configurations.
// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
//  Single closed-loop trial.
// ----------------------------------------------------------------------------
struct TrialResult
{
    double mass_kg          = 0;
    double Jxx              = 0;
    double Jyy              = 0;
    double Jzz              = 0;

    double tau_xy_lim       = 0;     // applied Limits::max_torque_xy_Nm
    double tau_z_lim        = 0;     // applied Limits::max_torque_z_Nm

    double max_pos_err_m    = 0;     // peak  ||pos - pos_des||  over whole run
    double rms_pos_err_m    = 0;     // RMS over Phase B + C
    double peak_tilt_deg    = 0;     // peak angle between body-z and world-z
    double peak_rpm         = 0;
    double final_pos_err_m  = 0;
    double final_z_m        = 0;

    double saturation_frac  = 0;     // fraction of ticks with any RPM ≥ 0.99·RPM_MAX
    double mean_residual    = 0;     // mean ||M·x - v_des|| (allocator residual)

    bool   diverged         = false;
    bool   hover_failed     = false;

    std::string verdict;
};

static TrialResult runTrial(double mass_kg,
                            const Eigen::Matrix3d& J,
                            bool verbose = false)
{
    // ---- 1.  Build a fresh stack ------------------------------------------
    coaxial_x8_dynamics dyn(mass_kg, J);
    Eigen::Vector3d p0(0.0, 0.0, 5.0);
    Eigen::Vector4d q0(1.0, 0.0, 0.0, 0.0);     // identity quat (w, x, y, z)
    dyn.init(p0, q0);

    CoaxialX8Controller ctrl(200.0);
    {
        CoaxialX8Controller::Params cp;
        cp.mass    = mass_kg;
        cp.inertia = J;
        cp.g       = 9.81;
        ctrl.setParams(cp);

        // ----- Adaptive torque-limit policy --------------------------------
        // The default 20 N·m hard cap is calibrated for the 1.9 kg baseline.
        // Scale linearly with diagonal inertia to maintain α_max ≈ 20 rad/s²,
        // capped at half the rotor envelope (so the inverse model has slack).
        constexpr double alpha_des  = 20.0;     // rad/s²
        constexpr double tau_xy_cap = 1500.0;   // N·m (≈ ½ aero hull)
        constexpr double tau_z_cap  =  175.0;   // N·m (≈ ½ aero hull)

        CoaxialX8Controller::Limits cl;         // start from defaults
        cl.max_torque_xy_Nm = std::min(alpha_des * J(0, 0), tau_xy_cap);
        cl.max_torque_z_Nm  = std::min(alpha_des * J(2, 2), tau_z_cap);
        ctrl.setLimits(cl);

        // We keep all other defaults (max_acc_xy = 10, max_tilt_rad = 30°,
        // etc.) — we are testing "the controller as configured" against
        // varying plant mass, only adapting the torque budget to the
        // physical plant.
    }

    CoaxialX8AllocatorQP::Params ap;
    ap.verbose = false;
    CoaxialX8AllocatorQP alloc(ap);

    // ---- 2.  Setpoint schedule -------------------------------------------
    auto setpointAt = [&](double t,
                          Eigen::Vector3d& p_des,
                          Eigen::Vector3d& v_des,
                          Eigen::Vector3d& a_des,
                          double& yaw_des)
    {
        v_des.setZero();
        a_des.setZero();
        yaw_des = 0.0;

        if (t < 5.0) {
            p_des << 0.0, 0.0, 5.0;            // Phase A: hover
        } else if (t < 15.0) {
            p_des << 2.0, 0.0, 5.0;            // Phase B: step forward
        } else {
            p_des << 0.0, 0.0, 5.0;            // Phase C: step back
        }
    };

    // ---- 3.  Time loop ----------------------------------------------------
    constexpr double T_total       = 25.0;
    constexpr double dt_ctrl       = 1.0 / 200.0;     // 200 Hz control
    constexpr int    N_DYN_PER_CTL = 5;                // → 1 kHz dynamics
    constexpr double dt_dyn        = dt_ctrl / N_DYN_PER_CTL;

    const int N_steps = static_cast<int>(T_total / dt_ctrl);

    TrialResult R;
    R.mass_kg = mass_kg;
    R.Jxx = J(0, 0); R.Jyy = J(1, 1); R.Jzz = J(2, 2);
    {
        // Mirror the limit policy used inside the controller, for logging.
        constexpr double alpha_des  = 20.0;
        constexpr double tau_xy_cap = 1500.0;
        constexpr double tau_z_cap  =  175.0;
        R.tau_xy_lim = std::min(alpha_des * J(0, 0), tau_xy_cap);
        R.tau_z_lim  = std::min(alpha_des * J(2, 2), tau_z_cap);
    }

    int    n_err_samples = 0;
    double sum_sq_err    = 0.0;
    int    n_sat         = 0;
    double sum_residual  = 0.0;
    int    n_residual    = 0;

    Eigen::Vector3d p_des, v_des, a_des;
    double yaw_des;

    for (int k = 0; k < N_steps; ++k) {
        const double t_now = k * dt_ctrl;

        // ---- Read plant state -------------------------------------------
        Eigen::Vector3d pos_w   = dyn.getPos();
        Eigen::Vector3d vel_w   = dyn.getVel();
        Eigen::Vector3d omega_b = dyn.getAngularVelBody();
        Eigen::Vector4d qv      = dyn.getQuat();
        Eigen::Quaterniond q(qv(0), qv(1), qv(2), qv(3));

        // ---- Divergence check -------------------------------------------
        if (!std::isfinite(pos_w.norm()) || pos_w.norm() > 1000.0 ||
            !std::isfinite(vel_w.norm()) || vel_w.norm() > 200.0)
        {
            R.diverged = true;
            R.verdict  = "DIVERGED";
            break;
        }

        // ---- Track current setpoint -------------------------------------
        setpointAt(t_now, p_des, v_des, a_des, yaw_des);

        const double err_norm = (pos_w - p_des).norm();
        R.max_pos_err_m = std::max(R.max_pos_err_m, err_norm);

        // RMS over Phase B + C only (avoids transient at t=0)
        if (t_now >= 5.0) {
            sum_sq_err += err_norm * err_norm;
            ++n_err_samples;
        }

        // Tilt = angle between body-z and world-z
        Eigen::Matrix3d Rmat = q.toRotationMatrix();
        const double cos_tilt = std::clamp(Rmat(2, 2), -1.0, 1.0);
        const double tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;
        R.peak_tilt_deg = std::max(R.peak_tilt_deg, tilt_deg);

        // ---- Run controller ---------------------------------------------
        ctrl.setFeedback(pos_w, vel_w, q, omega_b);
        ctrl.setSetpoint(p_des, v_des, a_des, yaw_des);
        auto u = ctrl.run();

        // ---- Run allocator ----------------------------------------------
        Eigen::Vector3d tau = u.torque_body;
        auto alloc_res = alloc.allocate(u.thrust_body_z, tau);
        sum_residual += alloc_res.residual;
        ++n_residual;

        const double rpm_max = CoaxialRotorModel::rpmMax();
        bool any_sat = false;
        for (int i = 0; i < 8; ++i) {
            const double r = alloc_res.rpm(i);
            R.peak_rpm = std::max(R.peak_rpm, r);
            if (r >= 0.995 * rpm_max) any_sat = true;
        }
        if (any_sat) ++n_sat;

        // ---- Apply to plant ---------------------------------------------
        dyn.setRPMCmd(alloc_res.rpm);
        for (int j = 0; j < N_DYN_PER_CTL; ++j) dyn.step_forward(dt_dyn);

        if (verbose && (k % 200 == 0)) {
            std::printf(
                "  t=%5.2f  pos=(%6.2f %6.2f %6.2f)  err=%5.2f  "
                "tilt=%5.1f° F=%7.1f τ=(%5.1f %5.1f %5.1f) rpm_pk=%6.0f\n",
                t_now, pos_w.x(), pos_w.y(), pos_w.z(), err_norm,
                tilt_deg, u.thrust_body_z, u.torque_body.x(),
                u.torque_body.y(), u.torque_body.z(), R.peak_rpm);
        }
    }

    // ---- 4.  Final metrics & verdict --------------------------------------
    if (!R.diverged) {
        Eigen::Vector3d pf = dyn.getPos();
        R.final_pos_err_m  = (pf - p_des).norm();
        R.final_z_m        = pf.z();
    }
    R.rms_pos_err_m    = (n_err_samples > 0)
                            ? std::sqrt(sum_sq_err / n_err_samples)
                            : 0.0;
    R.saturation_frac  = (n_residual > 0)
                            ? double(n_sat) / n_residual
                            : 0.0;
    R.mean_residual    = (n_residual > 0)
                            ? sum_residual / n_residual
                            : 0.0;

    // Hover-collapse: altitude well below the commanded 5 m
    if (!R.diverged && R.final_z_m < 3.0) R.hover_failed = true;

    if (R.verdict.empty()) {
        if (R.hover_failed)             R.verdict = "HOVER_FAILED";
        else if (R.peak_tilt_deg > 60)  R.verdict = "UNSTABLE";
        else if (R.rms_pos_err_m > 2.0) R.verdict = "POOR";
        else if (R.rms_pos_err_m > 0.5 ||
                 R.peak_tilt_deg > 25.0 ||
                 R.saturation_frac > 0.05)
                                        R.verdict = "DEGRADED";
        else                            R.verdict = "OK";
    }
    return R;
}

// ----------------------------------------------------------------------------
//  Pretty-printing helpers
// ----------------------------------------------------------------------------
static void printHeader()
{
    std::cout << "\n"
              << "  m[kg]  | Jxx    Jzz   | τ_xy   | max_err  rms_err"
              << " | tilt°  rpm_pk | sat%  resid | final_z |  verdict\n"
              << "  -------+--------------+--------+------------------"
              << "+--------------+-------------+---------+--------------\n";
}

static void printRow(const TrialResult& R)
{
    std::cout << std::fixed
              << "  " << std::setw(6) << std::setprecision(1) << R.mass_kg
              << " | "
              << std::setw(6) << std::setprecision(2) << R.Jxx << " "
              << std::setw(6) << std::setprecision(2) << R.Jzz
              << " | "
              << std::setw(6) << std::setprecision(1) << R.tau_xy_lim
              << " | "
              << std::setw(7) << std::setprecision(2) << R.max_pos_err_m << " "
              << std::setw(7) << std::setprecision(2) << R.rms_pos_err_m
              << " | "
              << std::setw(5) << std::setprecision(1) << R.peak_tilt_deg << " "
              << std::setw(7) << std::setprecision(0) << R.peak_rpm
              << " | "
              << std::setw(4) << std::setprecision(1)
              << 100.0 * R.saturation_frac << " "
              << std::setw(6) << std::setprecision(2) << R.mean_residual
              << " | "
              << std::setw(7) << std::setprecision(2) << R.final_z_m
              << " | "
              << R.verdict
              << (R.diverged ? "  [diverged]" : "")
              << (R.hover_failed ? "  [hover-fail]" : "")
              << "\n";
}

static void writeCsvRow(std::ofstream& f, const TrialResult& R,
                        const std::string& tag)
{
    f << tag                       << ","
      << R.mass_kg                 << ","
      << R.Jxx << "," << R.Jyy     << "," << R.Jzz << ","
      << R.tau_xy_lim              << ","
      << R.tau_z_lim               << ","
      << R.max_pos_err_m           << ","
      << R.rms_pos_err_m           << ","
      << R.peak_tilt_deg           << ","
      << R.peak_rpm                << ","
      << R.saturation_frac         << ","
      << R.mean_residual           << ","
      << R.final_z_m               << ","
      << (R.diverged ? 1 : 0)      << ","
      << (R.hover_failed ? 1 : 0)  << ","
      << R.verdict
      << "\n";
}

// ============================================================================
//  MAIN — runs the mass sweep and (optionally) the inertia-decoupled sub-test
// ============================================================================
int main(int argc, char** argv)
{
    // Defensive ROS init — controller's update path uses ROS_INFO_THROTTLE
    // (we never call it, but ros::init also bootstraps ros::Time).
    ros::init(argc, argv, "test_so3_mass_stability",
              ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler);

    bool run_decoupled = false;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--decoupled") run_decoupled = true;

    // ---- Theoretical reference values (printed for context) --------------
    constexpr double F_MAX_AERO    = 4.0 * 1333.6;     // ≈ 5334 N
    const     double m_hover_aero  = F_MAX_AERO / 9.81;
    const     double m_with_resv   = 0.85 * m_hover_aero;
    const     double m_30deg_tilt  = F_MAX_AERO * std::cos(30.0 * M_PI / 180.0)
                                   / 9.81;

    std::cout << "================================================================\n"
              << "  SO(3) Closed-Loop Mass-Stability Sweep  (T = 300 K aero data)\n"
              << "================================================================\n"
              << "\n  Theoretical thrust ceilings (CFD vertex extrapolated):\n"
              << "     F_max (4 pairs × 1333.6 N) = " << F_MAX_AERO << " N\n"
              << "     pure-hover ceiling         = "
              << std::fixed << std::setprecision(1) << m_hover_aero << " kg\n"
              << "     with 15% attitude reserve  = " << m_with_resv << " kg\n"
              << "     with 30° tilt requirement  = " << m_30deg_tilt << " kg\n"
              << "\n  Per-kg inertia coefficients used: "
              << "Jx=" << std::setprecision(3) << KJX
              << ", Jy=" << KJY << ", Jz=" << KJZ
              << "  (central-payload, F_body=" << F_BODY << ")\n";

    // ---- 1. MAIN: mass sweep with coupled inertia (J ∝ m) -----------------
    // std::vector<double> mass_set = {
    //     1.9,   5.0,  10.0,  25.0,  50.0,  75.0, 100.0,
    //    150.0, 200.0, 250.0, 300.0, 350.0, 400.0,
    //    425.0, 450.0, 475.0, 500.0, 525.0, 550.0, 600.0
    // };
    std::vector<double> mass_set;
    for(double i = 60; i < 600.0; i += 20)
        mass_set.push_back(i);

    std::ofstream csv("so3_mass_sweep.csv");
    csv << "test,mass_kg,Jxx,Jyy,Jzz,tau_xy_lim,tau_z_lim,"
        << "max_pos_err,rms_pos_err,peak_tilt_deg,"
        << "peak_rpm,saturation_frac,mean_residual,final_z,diverged,"
        << "hover_failed,verdict\n";

    std::cout << "\n[Test 1] Mass sweep, J = m · diag("
              << KJX << ", " << KJY << ", " << KJZ << ")\n";
    printHeader();

    double last_ok_mass     = -1;
    double last_degraded    = -1;
    double first_unstable   = -1;
    double first_hover_fail = -1;

    for (double m : mass_set) {
        Eigen::Matrix3d J = inertiaForMass(m);
        TrialResult R = runTrial(m, J, /*verbose=*/false);
        printRow(R);
        writeCsvRow(csv, R, "mass_sweep");

        if (R.verdict == "OK")        last_ok_mass = m;
        if (R.verdict == "OK" || R.verdict == "DEGRADED")
                                       last_degraded = m;
        if ((R.verdict == "UNSTABLE" || R.verdict == "DIVERGED") &&
            first_unstable < 0)        first_unstable = m;
        if (R.hover_failed && first_hover_fail < 0)
                                       first_hover_fail = m;
    }

    std::cout << "\n  ---------- summary ----------\n";
    std::cout << "  last mass with OK verdict:      "
              << (last_ok_mass > 0 ? std::to_string(last_ok_mass) + " kg"
                                   : "(none)") << "\n";
    std::cout << "  last mass still trackable:      "
              << (last_degraded > 0 ? std::to_string(last_degraded) + " kg"
                                    : "(none)") << "\n";
    std::cout << "  first mass UNSTABLE/DIVERGED:   "
              << (first_unstable > 0
                    ? std::to_string(first_unstable) + " kg"
                    : "(not reached in sweep)") << "\n";
    std::cout << "  first mass HOVER-FAILED:        "
              << (first_hover_fail > 0
                    ? std::to_string(first_hover_fail) + " kg"
                    : "(not reached in sweep)") << "\n";

    // ---- 2. OPTIONAL: inertia-decoupled sub-test --------------------------
    //
    //  At one fixed (manageable) mass, vary the inertia coefficient
    //  separately. This isolates "controller stiffness vs. plant inertia"
    //  from "thrust authority vs. weight". Insight: as J grows alone, the
    //  attitude loop's ω_n stays constant (because Kp scales with J), but
    //  the τ-saturation limit (20 N·m) cuts ever-deeper into the response.
    //
    if (run_decoupled) {
        std::cout << "\n[Test 2] Inertia-decoupled sweep at m = 50 kg\n";
        printHeader();

        const double m_fixed = 50.0;
        for (double k_scale : {0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0}) {
            Eigen::Matrix3d J = inertiaForMass(
                m_fixed, KJX * k_scale, KJY * k_scale, KJZ * k_scale);
            TrialResult R = runTrial(m_fixed, J);
            R.mass_kg = m_fixed;       // keep mass column readable
            printRow(R);
            writeCsvRow(csv, R,
                        "inertia_decoupled_x" + std::to_string(k_scale));
        }
    }

    csv.close();
    std::cout << "\n  → CSV written:  so3_mass_sweep.csv\n";
    std::cout << "================================================================\n";

    return 0;
}
