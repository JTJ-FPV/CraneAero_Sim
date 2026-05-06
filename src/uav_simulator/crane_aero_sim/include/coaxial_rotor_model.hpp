// #pragma once
// /**
//  * coaxial_rotor_model.hpp
//  *
//  */

// #include <algorithm>
// #include <array>
// #include <cmath>
// #include <cstddef>
// #include <limits>
// #include <string>
// #include <vector>

// // ============================================================
// //  CFD data — 26 scatter points
// //  Columns: |RPM_u|, |RPM_l|, T_u[N], T_l[N], Q_u[Nm], Q_l[Nm]
// //  Upper rotor rotates CCW (negative in CFD convention).
// //  Q_u > 0 (reaction on airframe = CW), Q_l < 0 (reaction = CCW).
// // ============================================================
// namespace coaxial_data {

// static constexpr int    N_PTS   = 41;
// // static constexpr int    N_PTS   = 28;
// // static constexpr int    N_PTS   = 26;
// static constexpr double RPM_MIN = 0.0;
// static constexpr double RPM_MAX = 2245.0;

// // {RPM_u, RPM_l, T_u, T_l, Q_u, Q_l}
// static constexpr double DATA[41][6] = {
//     // { RPM_u, RPM_l, T_u, T_l, Q_u, Q_l }
//     {       900,       900,   125.132580,    81.105949,    12.358169,   -10.733984 }, // case 1, Ttot=206.239, Qtot=1.624
//     {      1200,      1200,   224.270000,   145.214880,    22.129674,   -19.197406 }, // case 2, Ttot=369.485, Qtot=2.932
//     {      1500,      1500,   353.440150,   229.274210,    34.921762,   -30.330198 }, // case 3, Ttot=582.714, Qtot=4.592
//     {      1800,      1800,   513.528830,   331.959830,    50.927226,   -44.049811 }, // case 4, Ttot=845.489, Qtot=6.877
//     {      2000,      2000,   637.932360,   413.040250,    63.428166,   -54.918733 }, // case 5, Ttot=1050.973, Qtot=8.509
//     {      2245,      2245,   811.081450,   522.475910,    81.062837,   -69.918253 }, // case 6, Ttot=1333.557, Qtot=11.145
//     {      1250,      1150,   247.938290,   117.037640,    24.166430,   -16.818912 }, // case 7, Ttot=364.976, Qtot=7.348
//     {      1150,      1250,   202.560970,   175.753770,    20.205838,   -21.733090 }, // case 8, Ttot=378.315, Qtot=-1.527
//     {      1300,      1100,   273.515970,    90.084665,    26.304498,   -14.532271 }, // case 9, Ttot=363.601, Qtot=11.772
//     {      1100,      1300,   182.691300,   208.303990,    18.399870,   -24.376236 }, // case 10, Ttot=390.995, Qtot=-5.976
//     {      1900,      1700,   587.452270,   246.165390,    57.351666,   -36.837591 }, // case 11, Ttot=833.618, Qtot=20.514
//     {      1700,      1900,   447.600850,   426.438750,    44.972027,   -51.947832 }, // case 12, Ttot=874.040, Qtot=-6.976
//     {      2000,      1600,   669.728190,   163.669410,    64.246872,   -29.671059 }, // case 13, Ttot=833.398, Qtot=34.576
//     {      1600,      2000,   388.968800,   529.020820,    39.517460,   -60.326811 }, // case 14, Ttot=917.990, Qtot=-20.809
//     {      2245,      2045,   827.346120,   370.384200,    81.522135,   -54.591714 }, // case 15, Ttot=1197.730, Qtot=26.930
//     {      2045,      2245,   656.492080,   590.351620,    66.176033,   -73.231263 }, // case 16, Ttot=1246.844, Qtot=-7.055
//     {      1100,      1100,   187.926970,   121.699690,    18.543342,   -16.089813 }, // case 17, Ttot=309.627, Qtot=2.454
//     {      1300,      1300,   264.003990,   171.225620,    26.053965,   -22.640595 }, // case 18, Ttot=435.230, Qtot=3.413
//     {      1600,      1600,   403.306220,   260.905840,    39.898904,   -34.549132 }, // case 19, Ttot=664.212, Qtot=5.350
//     {      1700,      1700,   456.567310,   296.057970,    45.201558,   -39.218293 }, // case 20, Ttot=752.625, Qtot=5.983
//     {      1900,      1900,   573.981620,   370.878000,    56.999453,   -49.277867 }, // case 21, Ttot=944.860, Qtot=7.722
//     {      2045,      2045,   668.037980,   432.408960,    66.472972,   -57.537616 }, // case 22, Ttot=1100.447, Qtot=8.935
//     {      2145,      2145,   737.746250,   475.813380,    73.565959,   -63.501394 }, // case 23, Ttot=1213.560, Qtot=10.065
//     {      2145,      2245,   730.829910,   557.624320,    73.334510,   -71.659254 }, // case 24, Ttot=1288.454, Qtot=1.675
//     {      2245,      2145,   818.922480,   443.319010,    81.279914,   -61.934390 }, // case 25, Ttot=1262.241, Qtot=19.346
//     {         0,         0,     0.000000,     0.000000,     0.000000,     0.000000 }, // case 26, Ttot=0.000, Qtot=0.000
//     {      2245,         0,   940.000000,     0.000000,    88.000000,     0.000000 }, // case 27, Ttot=940.000, Qtot=88.000
//     {         0,      2245,     0.000000,   940.000000,     0.000000,   -88.000000 }, // case 28, Ttot=940.000, Qtot=-88.000
//     {       500,       500,    19.184941,    24.923556,     5.118933,    -5.098301 }, // case 29, Ttot=44.108, Qtot=0.021
//     {       700,       700,    75.362129,    48.982195,     7.461777,    -6.497985 }, // case 30, Ttot=124.344, Qtot=0.964
//     {       500,       900,    34.304671,   126.074950,     3.707119,   -12.503938 }, // case 31, Ttot=160.380, Qtot=-8.797
//     {      2070,      2070,   685.046020,   443.296070,    68.199062,   -59.014925 }, // case 33, Ttot=1128.342, Qtot=9.184
//     {      2050,      2150,   665.096380,   511.314190,    66.621335,   -65.457742 }, // case 34, Ttot=1176.411, Qtot=1.164
//     {      2150,      2050,   748.588420,   403.787330,    74.120686,   -56.367625 }, // case 35, Ttot=1152.376, Qtot=17.753
//     {      2100,      2200,   699.263690,   535.428430,    70.107467,   -68.679668 }, // case 36, Ttot=1234.692, Qtot=1.428
//     {      2200,      2100,   785.104160,   424.880910,    77.836790,   -59.321210 }, // case 37, Ttot=1209.985, Qtot=18.516
//     {      2200,      2200,   777.369550,   502.337590,    77.606439,   -67.081547 }, // case 38, Ttot=1279.707, Qtot=10.525
//     {      2000,      2245,   624.250160,   606.044370,    63.026588,   -73.986494 }, // case 39, Ttot=1230.295, Qtot=-10.960
//     {      2245,      1900,   841.122550,   271.274750,    81.882180,   -44.441345 }, // case 40, Ttot=1112.397, Qtot=37.441
//     {      1900,      2245,   557.067360,   639.004060,    56.480956,   -75.540290 }, // case 41, Ttot=1196.071, Qtot=-19.059
//     {       500,      2245,     0.442367,   912.465850,     1.772780,   -82.198833 } // case 42, Ttot=912.908, Qtot=-80.426
// };


// // static constexpr double DATA[28][6] = {
// //     {   0,    0,        0,        0,        0,        0},
// //     {2245,    0,    940.0,        0,     88.0,        0},
// //     {   0, 2245,        0,    940.0,        0,    -88.0},
// //     { 900,  900,  125.133,   81.106,  12.3582, -10.7340},
// //     {1200, 1200,  224.270,  145.215,  22.1297, -19.1974},
// //     {1500, 1500,  353.440,  229.274,  34.9218, -30.3302},
// //     {1800, 1800,  513.529,  331.960,  50.9272, -44.0498},
// //     {2000, 2000,  637.932,  413.040,  63.4282, -54.9187},
// //     {2245, 2245,  811.081,  522.476,  81.0628, -69.9183},
// //     {1250, 1150,  247.938,  117.038,  24.1664, -16.8189},
// //     {1150, 1250,  202.561,  175.754,  20.2058, -21.7331},
// //     {1300, 1100,  273.516,   90.085,  26.3045, -14.5323},
// //     {1100, 1300,  182.691,  208.304,  18.3999, -24.3762},
// //     {1900, 1700,  587.452,  246.165,  57.3517, -36.8376},
// //     {1700, 1900,  447.601,  426.439,  44.9720, -51.9478},
// //     {2000, 1600,  669.728,  163.669,  64.2469, -29.6711},
// //     {1600, 2000,  388.969,  529.021,  39.5175, -60.3268},
// //     {2245, 2045,  827.346,  370.384,  81.5221, -54.5917},
// //     {2045, 2245,  656.492,  590.352,  66.1760, -73.2313},
// //     {1100, 1100,  187.927,  121.700,  18.5433, -16.0898},
// //     {1300, 1300,  264.004,  171.226,  26.0540, -22.6406},
// //     {1600, 1600,  403.306,  260.906,  39.8989, -34.5491},
// //     {1700, 1700,  456.567,  296.058,  45.2016, -39.2183},
// //     {1900, 1900,  573.982,  370.878,  56.9995, -49.2779},
// //     {2045, 2045,  668.038,  432.409,  66.4730, -57.5376},
// //     {2145, 2145,  737.746,  475.813,  73.5660, -63.5014},
// //     {2145, 2245,  730.830,  557.624,  73.3345, -71.6593},
// //     {2245, 2145,  818.922,  443.319,  81.2799, -61.9344}
// // };

// // static constexpr double DATA[26][6] = {
// //     {   0,    0,        0,        0,        0,        0},
// //     { 900,  900,  125.133,   81.106,  12.3582, -10.7340},
// //     {1200, 1200,  224.270,  145.215,  22.1297, -19.1974},
// //     {1500, 1500,  353.440,  229.274,  34.9218, -30.3302},
// //     {1800, 1800,  513.529,  331.960,  50.9272, -44.0498},
// //     {2000, 2000,  637.932,  413.040,  63.4282, -54.9187},
// //     {2245, 2245,  811.081,  522.476,  81.0628, -69.9183},
// //     {1250, 1150,  247.938,  117.038,  24.1664, -16.8189},
// //     {1150, 1250,  202.561,  175.754,  20.2058, -21.7331},
// //     {1300, 1100,  273.516,   90.085,  26.3045, -14.5323},
// //     {1100, 1300,  182.691,  208.304,  18.3999, -24.3762},
// //     {1900, 1700,  587.452,  246.165,  57.3517, -36.8376},
// //     {1700, 1900,  447.601,  426.439,  44.9720, -51.9478},
// //     {2000, 1600,  669.728,  163.669,  64.2469, -29.6711},
// //     {1600, 2000,  388.969,  529.021,  39.5175, -60.3268},
// //     {2245, 2045,  827.346,  370.384,  81.5221, -54.5917},
// //     {2045, 2245,  656.492,  590.352,  66.1760, -73.2313},
// //     {1100, 1100,  187.927,  121.700,  18.5433, -16.0898},
// //     {1300, 1300,  264.004,  171.226,  26.0540, -22.6406},
// //     {1600, 1600,  403.306,  260.906,  39.8989, -34.5491},
// //     {1700, 1700,  456.567,  296.058,  45.2016, -39.2183},
// //     {1900, 1900,  573.982,  370.878,  56.9995, -49.2779},
// //     {2045, 2045,  668.038,  432.409,  66.4730, -57.5376},
// //     {2145, 2145,  737.746,  475.813,  73.5660, -63.5014},
// //     {2145, 2245,  730.830,  557.624,  73.3345, -71.6593},
// //     {2245, 2145,  818.922,  443.319,  81.2799, -61.9344}
// // };

// } // namespace coaxial_data

// // ============================================================
// //  2D Thin-Plate Spline RBF interpolator
// //  φ(r) = r² ln(r),  augmented with polynomial (1, x, y)
// // ============================================================
// class ThinPlateSpline2D {
// public:
//     ThinPlateSpline2D() = default;

//     void build(const std::vector<std::array<double, 2>>& pts,
//                const std::vector<double>& vals)
//     {
//         n_ = static_cast<int>(pts.size());
//         pts_ = pts;

//         const int N = n_ + 3;   // n RBF weights + 3 polynomial coeffs
//         std::vector<double> A(N * N, 0.0);
//         std::vector<double> rhs(N, 0.0);

//         // Fill RBF block and polynomial block
//         for (int i = 0; i < n_; ++i) {
//             for (int j = 0; j < n_; ++j) {
//                 A[i * N + j] = phi(dist(pts_[i], pts_[j]));
//             }
//             A[i * N + n_]     = 1.0;
//             A[i * N + n_ + 1] = pts_[i][0];
//             A[i * N + n_ + 2] = pts_[i][1];

//             A[n_       * N + i] = 1.0;
//             A[(n_ + 1) * N + i] = pts_[i][0];
//             A[(n_ + 2) * N + i] = pts_[i][1];

//             rhs[i] = vals[i];
//         }

//         w_ = solve(A, rhs, N);
//     }

//     /** Evaluate interpolated value at (x, y). */
//     double eval(double x, double y) const {
//         double v = w_[n_] + w_[n_ + 1] * x + w_[n_ + 2] * y;
//         for (int i = 0; i < n_; ++i) {
//             double r = std::sqrt(sqr(x - pts_[i][0]) + sqr(y - pts_[i][1]));
//             v += w_[i] * phi(r);
//         }
//         return v;
//     }

//     /** Partial derivatives ∂f/∂x, ∂f/∂y at (x, y). */
//     void gradient(double x, double y, double& dfdx, double& dfdy) const {
//         dfdx = w_[n_ + 1];
//         dfdy = w_[n_ + 2];
//         for (int i = 0; i < n_; ++i) {
//             double dx = x - pts_[i][0];
//             double dy = y - pts_[i][1];
//             double r2 = dx * dx + dy * dy;
//             if (r2 < 1e-24) continue;
//             double r = std::sqrt(r2);
//             // d/dx [r²ln(r)] = dx * (2*ln(r) + 1)
//             double coeff = w_[i] * (2.0 * std::log(r) + 1.0);
//             dfdx += coeff * dx;
//             dfdy += coeff * dy;
//         }
//     }

// private:
//     static double sqr(double v) { return v * v; }

//     static double phi(double r) {
//         return (r < 1e-12) ? 0.0 : r * r * std::log(r);
//     }

//     static double dist(const std::array<double, 2>& a,
//                        const std::array<double, 2>& b) {
//         return std::sqrt(sqr(a[0] - b[0]) + sqr(a[1] - b[1]));
//     }

//     /** Gaussian elimination with partial pivoting. */
//     static std::vector<double> solve(std::vector<double> A,
//                                      std::vector<double> b, int N)
//     {
//         for (int col = 0; col < N; ++col) {
//             int pivot = col;
//             for (int row = col + 1; row < N; ++row) {
//                 if (std::abs(A[row * N + col]) > std::abs(A[pivot * N + col]))
//                     pivot = row;
//             }
//             if (pivot != col) {
//                 for (int k = 0; k < N; ++k)
//                     std::swap(A[col * N + k], A[pivot * N + k]);
//                 std::swap(b[col], b[pivot]);
//             }
//             double diag = A[col * N + col];
//             if (std::abs(diag) < 1e-14) continue;
//             for (int row = col + 1; row < N; ++row) {
//                 double f = A[row * N + col] / diag;
//                 for (int k = col; k < N; ++k)
//                     A[row * N + k] -= f * A[col * N + k];
//                 b[row] -= f * b[col];
//             }
//         }
//         std::vector<double> x(N, 0.0);
//         for (int i = N - 1; i >= 0; --i) {
//             double s = b[i];
//             for (int j = i + 1; j < N; ++j) s -= A[i * N + j] * x[j];
//             x[i] = (std::abs(A[i * N + i]) > 1e-14) ? s / A[i * N + i] : 0.0;
//         }
//         return x;
//     }

//     int n_ = 0;
//     std::vector<std::array<double, 2>> pts_;
//     std::vector<double> w_;
// };

// // ============================================================
// //  Public result types
// // ============================================================

// /** Forward query result. */
// struct AeroQueryResult {
//     double T_u      = 0.0;   // upper propeller thrust  [N]
//     double T_l      = 0.0;   // lower propeller thrust  [N]
//     double Q_u      = 0.0;   // upper propeller torque   [N·m]  (> 0)
//     double Q_l      = 0.0;   // lower propeller torque   [N·m]  (< 0)
//     double T_total  = 0.0;   // T_u + T_l
//     double Q_total  = 0.0;   // Q_u + Q_l  (net yaw torque)
// };

// /** Inverse query result: (T_des, Q_des) → (RPM_u, RPM_l). */
// struct AeroInverseResult {
//     bool   success   = false;
//     double omega_u   = 0.0;   // solved |RPM| upper
//     double omega_l   = 0.0;   // solved |RPM| lower
//     double T_pred    = 0.0;   // T_total at solution
//     double Q_pred    = 0.0;   // Q_total at solution
//     double objective = 0.0;   // normalized residual (0 = perfect)
//     std::string status;       // "converged" / "max_iter" / "failed"
// };

// // ============================================================
// //  Main model class
// // ============================================================
// class CoaxialRotorModel {
// public:
//     /** Constructor — builds four RBF interpolators from embedded CFD data. */
//     CoaxialRotorModel() { buildFromData(); }

//     // --------------------------------------------------------
//     //  Forward query
//     // --------------------------------------------------------

//     /**
//      * Interpolate aerodynamic loads at given rotor speeds.
//      *
//      * @param rpm_u  Upper rotor speed magnitude [RPM],  clamped to [900, 2245]
//      * @param rpm_l  Lower rotor speed magnitude [RPM],  clamped to [900, 2245]
//      * @param clamp  If true (default), clamp inputs to data range.
//      *               If false, allow extrapolation (use with caution).
//      */
//     AeroQueryResult query(double rpm_u, double rpm_l, bool clamp = true) const {
//         if (clamp) {
//             rpm_u = std::clamp(std::abs(rpm_u),
//                                coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
//             rpm_l = std::clamp(std::abs(rpm_l),
//                                coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
//         } else {
//             rpm_u = std::abs(rpm_u);
//             rpm_l = std::abs(rpm_l);
//         }

//         AeroQueryResult r;
//         r.T_u     = rbf_Tu_.eval(rpm_u, rpm_l);
//         r.T_l     = rbf_Tl_.eval(rpm_u, rpm_l);
//         r.Q_u     = rbf_Qu_.eval(rpm_u, rpm_l);
//         r.Q_l     = rbf_Ql_.eval(rpm_u, rpm_l);
//         r.T_total = r.T_u + r.T_l;
//         r.Q_total = r.Q_u + r.Q_l;
//         return r;
//     }

//     /**
//      * Jacobian of [T_total, Q_total] w.r.t. [RPM_u, RPM_l].
//      * Useful for linearization and control design.
//      *
//      * Returns {{dT/du, dT/dl}, {dQ/du, dQ/dl}}.
//      */
//     std::array<std::array<double, 2>, 2>
//     jacobian(double rpm_u, double rpm_l) const {
//         rpm_u = std::clamp(std::abs(rpm_u),
//                            coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
//         rpm_l = std::clamp(std::abs(rpm_l),
//                            coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);

//         double dTu_du, dTu_dl, dTl_du, dTl_dl;
//         double dQu_du, dQu_dl, dQl_du, dQl_dl;

//         rbf_Tu_.gradient(rpm_u, rpm_l, dTu_du, dTu_dl);
//         rbf_Tl_.gradient(rpm_u, rpm_l, dTl_du, dTl_dl);
//         rbf_Qu_.gradient(rpm_u, rpm_l, dQu_du, dQu_dl);
//         rbf_Ql_.gradient(rpm_u, rpm_l, dQl_du, dQl_dl);

//         return {{
//             {dTu_du + dTl_du, dTu_dl + dTl_dl},   // dT_total / d(u,l)
//             {dQu_du + dQl_du, dQu_dl + dQl_dl}    // dQ_total / d(u,l)
//         }};
//     }

//     // --------------------------------------------------------
//     //  Inverse query  (T_des, Q_des) → (RPM_u, RPM_l)
//     // --------------------------------------------------------

//     /**
//      * Find rotor speeds that produce the desired total thrust and torque.
//      *
//      * Two-phase solver:
//      *   Phase 1 — Newton-Raphson using the analytic Jacobian (fast, C² RBF)
//      *   Phase 2 — Pattern search fallback if Newton fails to converge
//      *
//      * @param T_des   Desired total thrust  [N]
//      * @param Q_des   Desired total torque  [N·m]
//      * @param tol     Convergence tolerance on normalized residual (default 1e-8)
//      * @param max_iter Maximum Newton iterations (default 50)
//      */
//     AeroInverseResult invert(double T_des, double Q_des,
//                              double tol = 1e-8, int max_iter = 50) const
//     {
//         // ---------- Seed: nearest CFD sample in (T, Q) space ----------
//         double best_obj = std::numeric_limits<double>::infinity();
//         double init_u = 1500.0, init_l = 1500.0;

//         for (int i = 0; i < coaxial_data::N_PTS; ++i) {
//             double Ti = coaxial_data::DATA[i][2] + coaxial_data::DATA[i][3];
//             double Qi = coaxial_data::DATA[i][4] + coaxial_data::DATA[i][5];
//             double obj = normalizedObj(Ti, Qi, T_des, Q_des);
//             if (obj < best_obj) {
//                 best_obj = obj;
//                 init_u = coaxial_data::DATA[i][0];
//                 init_l = coaxial_data::DATA[i][1];
//             }
//         }

//         // ---------- Phase 1: Newton-Raphson ----------
//         AeroInverseResult res = newtonSolve(T_des, Q_des, init_u, init_l,
//                                             tol, max_iter);
//         if (res.success && res.objective < tol) {
//             return res;
//         }

//         // ---------- Phase 2: Pattern search fallback ----------
//         return patternSearch(T_des, Q_des,
//                              res.omega_u, res.omega_l, tol);
//     }

//     // --------------------------------------------------------
//     //  Accessors
//     // --------------------------------------------------------

//     static constexpr double rpmMin() { return coaxial_data::RPM_MIN; }
//     static constexpr double rpmMax() { return coaxial_data::RPM_MAX; }

// private:
//     ThinPlateSpline2D rbf_Tu_, rbf_Tl_, rbf_Qu_, rbf_Ql_;

//     // ---- Build from embedded CFD data ----
//     void buildFromData() {
//         std::vector<std::array<double, 2>> pts(coaxial_data::N_PTS);
//         std::vector<double> vTu(coaxial_data::N_PTS);
//         std::vector<double> vTl(coaxial_data::N_PTS);
//         std::vector<double> vQu(coaxial_data::N_PTS);
//         std::vector<double> vQl(coaxial_data::N_PTS);

//         for (int i = 0; i < coaxial_data::N_PTS; ++i) {
//             pts[i]  = {coaxial_data::DATA[i][0], coaxial_data::DATA[i][1]};
//             vTu[i]  = coaxial_data::DATA[i][2];
//             vTl[i]  = coaxial_data::DATA[i][3];
//             vQu[i]  = coaxial_data::DATA[i][4];
//             vQl[i]  = coaxial_data::DATA[i][5];
//         }

//         rbf_Tu_.build(pts, vTu);
//         rbf_Tl_.build(pts, vTl);
//         rbf_Qu_.build(pts, vQu);
//         rbf_Ql_.build(pts, vQl);
//     }

//     // ---- Normalized objective in (T, Q) space ----
//     static double normalizedObj(double Tp, double Qp,
//                                 double Td, double Qd)
//     {
//         double sT = std::max(1.0, std::abs(Td));
//         double sQ = std::max(1.0, std::abs(Qd));
//         double eT = (Tp - Td) / sT;
//         double eQ = (Qp - Qd) / sQ;
//         return eT * eT + eQ * eQ;
//     }

//     // ---- Newton-Raphson solver ----
//     AeroInverseResult newtonSolve(double Td, double Qd,
//                                   double u0, double l0,
//                                   double tol, int max_iter) const
//     {
//         double u = u0, l = l0;

//         AeroInverseResult res;
//         res.status = "max_iter";

//         for (int iter = 0; iter < max_iter; ++iter) {
//             auto fwd = query(u, l, false);
//             double fT = fwd.T_total - Td;
//             double fQ = fwd.Q_total - Qd;

//             double obj = normalizedObj(fwd.T_total, fwd.Q_total, Td, Qd);

//             if (obj < tol) {
//                 res.success   = true;
//                 res.omega_u   = std::clamp(u, coaxial_data::RPM_MIN,
//                                               coaxial_data::RPM_MAX);
//                 res.omega_l   = std::clamp(l, coaxial_data::RPM_MIN,
//                                               coaxial_data::RPM_MAX);
//                 res.T_pred    = fwd.T_total;
//                 res.Q_pred    = fwd.Q_total;
//                 res.objective = obj;
//                 res.status    = "converged";
//                 return res;
//             }

//             // Jacobian J = [[dT/du, dT/dl], [dQ/du, dQ/dl]]
//             auto J = jacobian(u, l);
//             double a11 = J[0][0], a12 = J[0][1];
//             double a21 = J[1][0], a22 = J[1][1];

//             double det = a11 * a22 - a12 * a21;
//             if (std::abs(det) < 1e-14) break;  // singular → fall to pattern search

//             // Newton step: [du, dl] = -J^{-1} * [fT, fQ]
//             double du = -(fT * a22 - fQ * a12) / det;
//             double dl = -(a11 * fQ - a21 * fT) / det;

//             // Damped step with line search (backtracking)
//             double alpha = 1.0;
//             for (int ls = 0; ls < 10; ++ls) {
//                 double u_new = u + alpha * du;
//                 double l_new = l + alpha * dl;
//                 // Soft boundary: allow slight overshoot, clamp later
//                 auto fwd_new = query(u_new, l_new, false);
//                 double obj_new = normalizedObj(fwd_new.T_total, fwd_new.Q_total,
//                                                Td, Qd);
//                 if (obj_new < obj) {
//                     u = u_new;
//                     l = l_new;
//                     break;
//                 }
//                 alpha *= 0.5;
//             }
//         }

//         // Did not converge — return best point found
//         u = std::clamp(u, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
//         l = std::clamp(l, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
//         auto fwd = query(u, l);
//         res.success   = false;
//         res.omega_u   = u;
//         res.omega_l   = l;
//         res.T_pred    = fwd.T_total;
//         res.Q_pred    = fwd.Q_total;
//         res.objective = normalizedObj(fwd.T_total, fwd.Q_total, Td, Qd);
//         return res;
//     }

//     // ---- Pattern search fallback (derivative-free) ----
//     AeroInverseResult patternSearch(double Td, double Qd,
//                                     double u0, double l0,
//                                     double tol) const
//     {
//         double u = std::clamp(u0, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
//         double l = std::clamp(l0, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);

//         auto fwd = query(u, l);
//         double best_obj = normalizedObj(fwd.T_total, fwd.Q_total, Td, Qd);

//         double step_u = 0.2 * (coaxial_data::RPM_MAX - coaxial_data::RPM_MIN);
//         double step_l = step_u;

//         for (int outer = 0; outer < 40; ++outer) {
//             bool improved = false;

//             for (int du = -1; du <= 1; ++du) {
//                 for (int dl = -1; dl <= 1; ++dl) {
//                     double cu = std::clamp(u + du * step_u,
//                                            coaxial_data::RPM_MIN,
//                                            coaxial_data::RPM_MAX);
//                     double cl = std::clamp(l + dl * step_l,
//                                            coaxial_data::RPM_MIN,
//                                            coaxial_data::RPM_MAX);
//                     auto qr = query(cu, cl);
//                     double obj = normalizedObj(qr.T_total, qr.Q_total, Td, Qd);
//                     if (obj < best_obj) {
//                         best_obj = obj;
//                         u = cu;
//                         l = cl;
//                         fwd = qr;
//                         improved = true;
//                     }
//                 }
//             }

//             if (!improved) {
//                 step_u *= 0.5;
//                 step_l *= 0.5;
//             }
//             if (step_u < 0.5 && step_l < 0.5) break;
//             if (best_obj < tol) break;
//         }

//         AeroInverseResult res;
//         res.success   = (best_obj < 1e-4);
//         res.omega_u   = u;
//         res.omega_l   = l;
//         res.T_pred    = fwd.T_total;
//         res.Q_pred    = fwd.Q_total;
//         res.objective = best_obj;
//         res.status    = (best_obj < tol) ? "converged" : "pattern_search";
//         return res;
//     }
// };


#pragma once
/**
 * coaxial_rotor_model.hpp
 *
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

// ============================================================
//  CFD data — 26 scatter points
//  Columns: |RPM_u|, |RPM_l|, T_u[N], T_l[N], Q_u[Nm], Q_l[Nm]
//  Upper rotor rotates CCW (negative in CFD convention).
//  Q_u > 0 (reaction on airframe = CW), Q_l < 0 (reaction = CCW).
// ============================================================
namespace coaxial_data {

static constexpr int    N_PTS   = 41;
// static constexpr int    N_PTS   = 28;
// static constexpr int    N_PTS   = 26;
static constexpr double RPM_MIN = 0.0;
static constexpr double RPM_MAX = 2245.0;

// {RPM_u, RPM_l, T_u, T_l, Q_u, Q_l}
static constexpr double DATA[41][6] = {
    // { RPM_u, RPM_l, T_u, T_l, Q_u, Q_l }
    {       900,       900,   125.132580,    81.105949,    12.358169,   -10.733984 }, // case 1, Ttot=206.239, Qtot=1.624
    {      1200,      1200,   224.270000,   145.214880,    22.129674,   -19.197406 }, // case 2, Ttot=369.485, Qtot=2.932
    {      1500,      1500,   353.440150,   229.274210,    34.921762,   -30.330198 }, // case 3, Ttot=582.714, Qtot=4.592
    {      1800,      1800,   513.528830,   331.959830,    50.927226,   -44.049811 }, // case 4, Ttot=845.489, Qtot=6.877
    {      2000,      2000,   637.932360,   413.040250,    63.428166,   -54.918733 }, // case 5, Ttot=1050.973, Qtot=8.509
    {      2245,      2245,   811.081450,   522.475910,    81.062837,   -69.918253 }, // case 6, Ttot=1333.557, Qtot=11.145
    {      1250,      1150,   247.938290,   117.037640,    24.166430,   -16.818912 }, // case 7, Ttot=364.976, Qtot=7.348
    {      1150,      1250,   202.560970,   175.753770,    20.205838,   -21.733090 }, // case 8, Ttot=378.315, Qtot=-1.527
    {      1300,      1100,   273.515970,    90.084665,    26.304498,   -14.532271 }, // case 9, Ttot=363.601, Qtot=11.772
    {      1100,      1300,   182.691300,   208.303990,    18.399870,   -24.376236 }, // case 10, Ttot=390.995, Qtot=-5.976
    {      1900,      1700,   587.452270,   246.165390,    57.351666,   -36.837591 }, // case 11, Ttot=833.618, Qtot=20.514
    {      1700,      1900,   447.600850,   426.438750,    44.972027,   -51.947832 }, // case 12, Ttot=874.040, Qtot=-6.976
    {      2000,      1600,   669.728190,   163.669410,    64.246872,   -29.671059 }, // case 13, Ttot=833.398, Qtot=34.576
    {      1600,      2000,   388.968800,   529.020820,    39.517460,   -60.326811 }, // case 14, Ttot=917.990, Qtot=-20.809
    {      2245,      2045,   827.346120,   370.384200,    81.522135,   -54.591714 }, // case 15, Ttot=1197.730, Qtot=26.930
    {      2045,      2245,   656.492080,   590.351620,    66.176033,   -73.231263 }, // case 16, Ttot=1246.844, Qtot=-7.055
    {      1100,      1100,   187.926970,   121.699690,    18.543342,   -16.089813 }, // case 17, Ttot=309.627, Qtot=2.454
    {      1300,      1300,   264.003990,   171.225620,    26.053965,   -22.640595 }, // case 18, Ttot=435.230, Qtot=3.413
    {      1600,      1600,   403.306220,   260.905840,    39.898904,   -34.549132 }, // case 19, Ttot=664.212, Qtot=5.350
    {      1700,      1700,   456.567310,   296.057970,    45.201558,   -39.218293 }, // case 20, Ttot=752.625, Qtot=5.983
    {      1900,      1900,   573.981620,   370.878000,    56.999453,   -49.277867 }, // case 21, Ttot=944.860, Qtot=7.722
    {      2045,      2045,   668.037980,   432.408960,    66.472972,   -57.537616 }, // case 22, Ttot=1100.447, Qtot=8.935
    {      2145,      2145,   737.746250,   475.813380,    73.565959,   -63.501394 }, // case 23, Ttot=1213.560, Qtot=10.065
    {      2145,      2245,   730.829910,   557.624320,    73.334510,   -71.659254 }, // case 24, Ttot=1288.454, Qtot=1.675
    {      2245,      2145,   818.922480,   443.319010,    81.279914,   -61.934390 }, // case 25, Ttot=1262.241, Qtot=19.346
    {         0,         0,     0.000000,     0.000000,     0.000000,     0.000000 }, // case 26, Ttot=0.000, Qtot=0.000
    {      2245,         0,   940.000000,     0.000000,    88.000000,     0.000000 }, // case 27, Ttot=940.000, Qtot=88.000
    {         0,      2245,     0.000000,   940.000000,     0.000000,   -88.000000 }, // case 28, Ttot=940.000, Qtot=-88.000
    {       500,       500,    19.184941,    24.923556,     5.118933,    -5.098301 }, // case 29, Ttot=44.108, Qtot=0.021
    {       700,       700,    75.362129,    48.982195,     7.461777,    -6.497985 }, // case 30, Ttot=124.344, Qtot=0.964
    {       500,       900,    34.304671,   126.074950,     3.707119,   -12.503938 }, // case 31, Ttot=160.380, Qtot=-8.797
    {      2070,      2070,   685.046020,   443.296070,    68.199062,   -59.014925 }, // case 33, Ttot=1128.342, Qtot=9.184
    {      2050,      2150,   665.096380,   511.314190,    66.621335,   -65.457742 }, // case 34, Ttot=1176.411, Qtot=1.164
    {      2150,      2050,   748.588420,   403.787330,    74.120686,   -56.367625 }, // case 35, Ttot=1152.376, Qtot=17.753
    {      2100,      2200,   699.263690,   535.428430,    70.107467,   -68.679668 }, // case 36, Ttot=1234.692, Qtot=1.428
    {      2200,      2100,   785.104160,   424.880910,    77.836790,   -59.321210 }, // case 37, Ttot=1209.985, Qtot=18.516
    {      2200,      2200,   777.369550,   502.337590,    77.606439,   -67.081547 }, // case 38, Ttot=1279.707, Qtot=10.525
    {      2000,      2245,   624.250160,   606.044370,    63.026588,   -73.986494 }, // case 39, Ttot=1230.295, Qtot=-10.960
    {      2245,      1900,   841.122550,   271.274750,    81.882180,   -44.441345 }, // case 40, Ttot=1112.397, Qtot=37.441
    {      1900,      2245,   557.067360,   639.004060,    56.480956,   -75.540290 } // case 41, Ttot=1196.071, Qtot=-19.059
    // {       500,      2245,     0.442367,   912.465850,     1.772780,   -82.198833 }  // case 42, Ttot=912.908, Qtot=-80.426
};


// static constexpr double DATA[28][6] = {
//     {   0,    0,        0,        0,        0,        0},
//     {2245,    0,    940.0,        0,     88.0,        0},
//     {   0, 2245,        0,    940.0,        0,    -88.0},
//     { 900,  900,  125.133,   81.106,  12.3582, -10.7340},
//     {1200, 1200,  224.270,  145.215,  22.1297, -19.1974},
//     {1500, 1500,  353.440,  229.274,  34.9218, -30.3302},
//     {1800, 1800,  513.529,  331.960,  50.9272, -44.0498},
//     {2000, 2000,  637.932,  413.040,  63.4282, -54.9187},
//     {2245, 2245,  811.081,  522.476,  81.0628, -69.9183},
//     {1250, 1150,  247.938,  117.038,  24.1664, -16.8189},
//     {1150, 1250,  202.561,  175.754,  20.2058, -21.7331},
//     {1300, 1100,  273.516,   90.085,  26.3045, -14.5323},
//     {1100, 1300,  182.691,  208.304,  18.3999, -24.3762},
//     {1900, 1700,  587.452,  246.165,  57.3517, -36.8376},
//     {1700, 1900,  447.601,  426.439,  44.9720, -51.9478},
//     {2000, 1600,  669.728,  163.669,  64.2469, -29.6711},
//     {1600, 2000,  388.969,  529.021,  39.5175, -60.3268},
//     {2245, 2045,  827.346,  370.384,  81.5221, -54.5917},
//     {2045, 2245,  656.492,  590.352,  66.1760, -73.2313},
//     {1100, 1100,  187.927,  121.700,  18.5433, -16.0898},
//     {1300, 1300,  264.004,  171.226,  26.0540, -22.6406},
//     {1600, 1600,  403.306,  260.906,  39.8989, -34.5491},
//     {1700, 1700,  456.567,  296.058,  45.2016, -39.2183},
//     {1900, 1900,  573.982,  370.878,  56.9995, -49.2779},
//     {2045, 2045,  668.038,  432.409,  66.4730, -57.5376},
//     {2145, 2145,  737.746,  475.813,  73.5660, -63.5014},
//     {2145, 2245,  730.830,  557.624,  73.3345, -71.6593},
//     {2245, 2145,  818.922,  443.319,  81.2799, -61.9344}
// };

// static constexpr double DATA[26][6] = {
//     {   0,    0,        0,        0,        0,        0},
//     { 900,  900,  125.133,   81.106,  12.3582, -10.7340},
//     {1200, 1200,  224.270,  145.215,  22.1297, -19.1974},
//     {1500, 1500,  353.440,  229.274,  34.9218, -30.3302},
//     {1800, 1800,  513.529,  331.960,  50.9272, -44.0498},
//     {2000, 2000,  637.932,  413.040,  63.4282, -54.9187},
//     {2245, 2245,  811.081,  522.476,  81.0628, -69.9183},
//     {1250, 1150,  247.938,  117.038,  24.1664, -16.8189},
//     {1150, 1250,  202.561,  175.754,  20.2058, -21.7331},
//     {1300, 1100,  273.516,   90.085,  26.3045, -14.5323},
//     {1100, 1300,  182.691,  208.304,  18.3999, -24.3762},
//     {1900, 1700,  587.452,  246.165,  57.3517, -36.8376},
//     {1700, 1900,  447.601,  426.439,  44.9720, -51.9478},
//     {2000, 1600,  669.728,  163.669,  64.2469, -29.6711},
//     {1600, 2000,  388.969,  529.021,  39.5175, -60.3268},
//     {2245, 2045,  827.346,  370.384,  81.5221, -54.5917},
//     {2045, 2245,  656.492,  590.352,  66.1760, -73.2313},
//     {1100, 1100,  187.927,  121.700,  18.5433, -16.0898},
//     {1300, 1300,  264.004,  171.226,  26.0540, -22.6406},
//     {1600, 1600,  403.306,  260.906,  39.8989, -34.5491},
//     {1700, 1700,  456.567,  296.058,  45.2016, -39.2183},
//     {1900, 1900,  573.982,  370.878,  56.9995, -49.2779},
//     {2045, 2045,  668.038,  432.409,  66.4730, -57.5376},
//     {2145, 2145,  737.746,  475.813,  73.5660, -63.5014},
//     {2145, 2245,  730.830,  557.624,  73.3345, -71.6593},
//     {2245, 2145,  818.922,  443.319,  81.2799, -61.9344}
// };

} // namespace coaxial_data

// ============================================================
//  2D Cubic RBF Interpolator (with linear polynomial tail)
//
//  Kernel:        φ(r) = r³        (polyharmonic, conditionally
//                                    positive definite of order 2)
//  Augmentation:  { 1, x, y }      (required for c.p.d. order 2)
//
//  This is an EXACT interpolator: f(x_i, y_i) = v_i at every CFD
//  sample.  Unlike Gaussian or multiquadric RBFs, the cubic kernel
//  has no tunable shape parameter and no decay length — it gives
//  the most rigid, "stiffest" extrapolation behaviour of all
//  positive-definite kernels.
//
//  Selected because it gave the lowest strict-extrapolation RMSE
//  (43.4) of any method in the offline benchmark — about half
//  that of the previous Thin-Plate Spline (87.9) and about a
//  third that of Polynomial-2 (62.9).
//
//  Numerical conditioning:
//      The collocation matrix is
//          [ K  P ]      with K_ij = |x_i - x_j|³
//          [ Pᵀ 0 ],          P_i  = [1, x_i, y_i].
//      In raw RPM, K reaches ~3·10¹⁰ while P reaches ~2245, mixing
//      ~10 orders of magnitude into one matrix.  We therefore
//      internally normalise inputs:
//          x_n = x / S, y_n = y / S, S = RPM_MAX = 2245.
//      The cubic kernel is shape-invariant under uniform scaling
//      ( φ(αr) = α³ φ(r) just rescales weights ), so predictions
//      at any query point are mathematically identical to a fit
//      on raw RPM — only the conditioning improves.
// ============================================================
class RBFCubic2D {
public:
    RBFCubic2D() = default;

    void build(const std::vector<std::array<double, 2>>& pts,
               const std::vector<double>& vals)
    {
        n_ = static_cast<int>(pts.size());
        pts_n_.resize(n_);
        for (int i = 0; i < n_; ++i) {
            pts_n_[i] = {pts[i][0] * INV_SCALE, pts[i][1] * INV_SCALE};
        }

        const int N = n_ + 3;        // n RBF weights + 3 polynomial coeffs
        std::vector<double> A(static_cast<std::size_t>(N) * N, 0.0);
        std::vector<double> rhs(N, 0.0);

        // ---- Fill RBF block + polynomial augmentation (normalised space) ----
        for (int i = 0; i < n_; ++i) {
            for (int j = 0; j < n_; ++j) {
                A[i * N + j] = phi(dist(pts_n_[i], pts_n_[j]));
            }
            A[i * N + n_]     = 1.0;
            A[i * N + n_ + 1] = pts_n_[i][0];
            A[i * N + n_ + 2] = pts_n_[i][1];

            A[ n_      * N + i] = 1.0;
            A[(n_ + 1) * N + i] = pts_n_[i][0];
            A[(n_ + 2) * N + i] = pts_n_[i][1];

            rhs[i] = vals[i];
        }

        w_ = solve(A, rhs, N);
    }

    /** Evaluate the surrogate at (x, y) in physical (RPM) units. */
    double eval(double x, double y) const {
        const double xn = x * INV_SCALE;
        const double yn = y * INV_SCALE;
        double v = w_[n_] + w_[n_ + 1] * xn + w_[n_ + 2] * yn;
        for (int i = 0; i < n_; ++i) {
            const double dx = xn - pts_n_[i][0];
            const double dy = yn - pts_n_[i][1];
            const double r  = std::sqrt(dx * dx + dy * dy);
            v += w_[i] * phi(r);
        }
        return v;
    }

    /** Partial derivatives ∂f/∂x, ∂f/∂y in physical (un-normalised) units.
     *  Cubic RBF gradient:  d/dxn [r³] = 3 r · (xn - xi_n)
     *  Chain rule for physical units:  ∂/∂x = (1/S) · ∂/∂xn. */
    void gradient(double x, double y, double& dfdx, double& dfdy) const {
        const double xn = x * INV_SCALE;
        const double yn = y * INV_SCALE;

        // Polynomial part (in normalised coordinates):
        double dxn = w_[n_ + 1];
        double dyn = w_[n_ + 2];

        // RBF part — note: at r = 0 the term is naturally zero (dx = dy = 0),
        // so unlike the TPS r²log(r) kernel there is no log singularity.
        for (int i = 0; i < n_; ++i) {
            const double dx = xn - pts_n_[i][0];
            const double dy = yn - pts_n_[i][1];
            const double r2 = dx * dx + dy * dy;
            if (r2 < 1e-24) continue;
            const double r = std::sqrt(r2);
            const double coeff = 3.0 * w_[i] * r;
            dxn += coeff * dx;
            dyn += coeff * dy;
        }

        dfdx = dxn * INV_SCALE;
        dfdy = dyn * INV_SCALE;
    }

private:
    static constexpr double SCALE     = coaxial_data::RPM_MAX;   // 2245
    static constexpr double INV_SCALE = 1.0 / SCALE;

    static double sqr(double v) { return v * v; }

    /** Cubic RBF kernel φ(r) = r³ — C∞ smooth except for a derivative
     *  discontinuity of order 4 at r=0, which does not affect us. */
    static double phi(double r) {
        return r * r * r;
    }

    static double dist(const std::array<double, 2>& a,
                       const std::array<double, 2>& b) {
        return std::sqrt(sqr(a[0] - b[0]) + sqr(a[1] - b[1]));
    }

    /** Gaussian elimination with partial pivoting. */
    static std::vector<double> solve(std::vector<double> A,
                                     std::vector<double> b, int N)
    {
        for (int col = 0; col < N; ++col) {
            int pivot = col;
            for (int row = col + 1; row < N; ++row) {
                if (std::abs(A[row * N + col]) > std::abs(A[pivot * N + col]))
                    pivot = row;
            }
            if (pivot != col) {
                for (int k = 0; k < N; ++k)
                    std::swap(A[col * N + k], A[pivot * N + k]);
                std::swap(b[col], b[pivot]);
            }
            double diag = A[col * N + col];
            if (std::abs(diag) < 1e-14) continue;
            for (int row = col + 1; row < N; ++row) {
                double f = A[row * N + col] / diag;
                for (int k = col; k < N; ++k)
                    A[row * N + k] -= f * A[col * N + k];
                b[row] -= f * b[col];
            }
        }
        std::vector<double> x(N, 0.0);
        for (int i = N - 1; i >= 0; --i) {
            double s = b[i];
            for (int j = i + 1; j < N; ++j) s -= A[i * N + j] * x[j];
            x[i] = (std::abs(A[i * N + i]) > 1e-14) ? s / A[i * N + i] : 0.0;
        }
        return x;
    }

    int n_ = 0;
    std::vector<std::array<double, 2>> pts_n_;   // points in normalised space
    std::vector<double> w_;                      // n RBF weights + 3 poly coeffs
};

// ============================================================
//  Public result types
// ============================================================

/** Forward query result. */
struct AeroQueryResult {
    double T_u      = 0.0;   // upper propeller thrust  [N]
    double T_l      = 0.0;   // lower propeller thrust  [N]
    double Q_u      = 0.0;   // upper propeller torque   [N·m]  (> 0)
    double Q_l      = 0.0;   // lower propeller torque   [N·m]  (< 0)
    double T_total  = 0.0;   // T_u + T_l
    double Q_total  = 0.0;   // Q_u + Q_l  (net yaw torque)
};

/** Inverse query result: (T_des, Q_des) → (RPM_u, RPM_l). */
struct AeroInverseResult {
    bool   success   = false;
    double omega_u   = 0.0;   // solved |RPM| upper
    double omega_l   = 0.0;   // solved |RPM| lower
    double T_pred    = 0.0;   // T_total at solution
    double Q_pred    = 0.0;   // Q_total at solution
    double objective = 0.0;   // normalized residual (0 = perfect)
    std::string status;       // "converged" / "max_iter" / "failed"
};

// ============================================================
//  Main model class
// ============================================================
class CoaxialRotorModel {
public:
    /** Constructor — fits four 2-D cubic-RBF interpolators
     *  (one per output channel: T_u, T_l, Q_u, Q_l) from the
     *  embedded CFD data set.  Cubic RBF passes through every
     *  CFD sample exactly. */
    CoaxialRotorModel() { buildFromData(); }

    // --------------------------------------------------------
    //  Forward query
    // --------------------------------------------------------

    /**
     * Interpolate aerodynamic loads at given rotor speeds.
     *
     * @param rpm_u  Upper rotor speed magnitude [RPM],  clamped to [900, 2245]
     * @param rpm_l  Lower rotor speed magnitude [RPM],  clamped to [900, 2245]
     * @param clamp  If true (default), clamp inputs to data range.
     *               If false, allow extrapolation (use with caution).
     */
    AeroQueryResult query(double rpm_u, double rpm_l, bool clamp = true) const {
        if (clamp) {
            rpm_u = std::clamp(std::abs(rpm_u),
                               coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
            rpm_l = std::clamp(std::abs(rpm_l),
                               coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
        } else {
            rpm_u = std::abs(rpm_u);
            rpm_l = std::abs(rpm_l);
        }

        AeroQueryResult r;
        r.T_u     = rbf_Tu_.eval(rpm_u, rpm_l);
        r.T_l     = rbf_Tl_.eval(rpm_u, rpm_l);
        r.Q_u     = rbf_Qu_.eval(rpm_u, rpm_l);
        r.Q_l     = rbf_Ql_.eval(rpm_u, rpm_l);
        r.T_total = r.T_u + r.T_l;
        r.Q_total = r.Q_u + r.Q_l;
        return r;
    }

    /**
     * Jacobian of [T_total, Q_total] w.r.t. [RPM_u, RPM_l].
     * Useful for linearization and control design.
     *
     * Returns {{dT/du, dT/dl}, {dQ/du, dQ/dl}}.
     */
    std::array<std::array<double, 2>, 2>
    jacobian(double rpm_u, double rpm_l) const {
        rpm_u = std::clamp(std::abs(rpm_u),
                           coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
        rpm_l = std::clamp(std::abs(rpm_l),
                           coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);

        double dTu_du, dTu_dl, dTl_du, dTl_dl;
        double dQu_du, dQu_dl, dQl_du, dQl_dl;

        rbf_Tu_.gradient(rpm_u, rpm_l, dTu_du, dTu_dl);
        rbf_Tl_.gradient(rpm_u, rpm_l, dTl_du, dTl_dl);
        rbf_Qu_.gradient(rpm_u, rpm_l, dQu_du, dQu_dl);
        rbf_Ql_.gradient(rpm_u, rpm_l, dQl_du, dQl_dl);

        return {{
            {dTu_du + dTl_du, dTu_dl + dTl_dl},   // dT_total / d(u,l)
            {dQu_du + dQl_du, dQu_dl + dQl_dl}    // dQ_total / d(u,l)
        }};
    }

    // --------------------------------------------------------
    //  Inverse query  (T_des, Q_des) → (RPM_u, RPM_l)
    // --------------------------------------------------------

    /**
     * Find rotor speeds that produce the desired total thrust and torque.
     *
     * Two-phase solver:
     *   Phase 1 — Newton-Raphson using the analytic Jacobian
     *             (Cubic-RBF surrogate is C¹ smooth, gradient is closed-form)
     *   Phase 2 — Pattern search fallback if Newton fails to converge
     *
     * @param T_des   Desired total thrust  [N]
     * @param Q_des   Desired total torque  [N·m]
     * @param tol     Convergence tolerance on normalized residual (default 1e-8)
     * @param max_iter Maximum Newton iterations (default 50)
     */
    AeroInverseResult invert(double T_des, double Q_des,
                             double tol = 1e-8, int max_iter = 50) const
    {
        // ---------- Seed: nearest CFD sample in (T, Q) space ----------
        double best_obj = std::numeric_limits<double>::infinity();
        double init_u = 1500.0, init_l = 1500.0;

        for (int i = 0; i < coaxial_data::N_PTS; ++i) {
            double Ti = coaxial_data::DATA[i][2] + coaxial_data::DATA[i][3];
            double Qi = coaxial_data::DATA[i][4] + coaxial_data::DATA[i][5];
            double obj = normalizedObj(Ti, Qi, T_des, Q_des);
            if (obj < best_obj) {
                best_obj = obj;
                init_u = coaxial_data::DATA[i][0];
                init_l = coaxial_data::DATA[i][1];
            }
        }

        // ---------- Phase 1: Newton-Raphson ----------
        AeroInverseResult res = newtonSolve(T_des, Q_des, init_u, init_l,
                                            tol, max_iter);
        if (res.success && res.objective < tol) {
            return res;
        }

        // ---------- Phase 2: Pattern search fallback ----------
        return patternSearch(T_des, Q_des,
                             res.omega_u, res.omega_l, tol);
    }

    // --------------------------------------------------------
    //  Accessors
    // --------------------------------------------------------

    static constexpr double rpmMin() { return coaxial_data::RPM_MIN; }
    static constexpr double rpmMax() { return coaxial_data::RPM_MAX; }

private:
    RBFCubic2D rbf_Tu_, rbf_Tl_, rbf_Qu_, rbf_Ql_;

    // ---- Build from embedded CFD data ----
    void buildFromData() {
        std::vector<std::array<double, 2>> pts(coaxial_data::N_PTS);
        std::vector<double> vTu(coaxial_data::N_PTS);
        std::vector<double> vTl(coaxial_data::N_PTS);
        std::vector<double> vQu(coaxial_data::N_PTS);
        std::vector<double> vQl(coaxial_data::N_PTS);

        for (int i = 0; i < coaxial_data::N_PTS; ++i) {
            pts[i]  = {coaxial_data::DATA[i][0], coaxial_data::DATA[i][1]};
            vTu[i]  = coaxial_data::DATA[i][2];
            vTl[i]  = coaxial_data::DATA[i][3];
            vQu[i]  = coaxial_data::DATA[i][4];
            vQl[i]  = coaxial_data::DATA[i][5];
        }

        rbf_Tu_.build(pts, vTu);
        rbf_Tl_.build(pts, vTl);
        rbf_Qu_.build(pts, vQu);
        rbf_Ql_.build(pts, vQl);
    }

    // ---- Normalized objective in (T, Q) space ----
    static double normalizedObj(double Tp, double Qp,
                                double Td, double Qd)
    {
        double sT = std::max(1.0, std::abs(Td));
        double sQ = std::max(1.0, std::abs(Qd));
        double eT = (Tp - Td) / sT;
        double eQ = (Qp - Qd) / sQ;
        return eT * eT + eQ * eQ;
    }

    // ---- Newton-Raphson solver ----
    AeroInverseResult newtonSolve(double Td, double Qd,
                                  double u0, double l0,
                                  double tol, int max_iter) const
    {
        double u = u0, l = l0;

        AeroInverseResult res;
        res.status = "max_iter";

        for (int iter = 0; iter < max_iter; ++iter) {
            auto fwd = query(u, l, false);
            double fT = fwd.T_total - Td;
            double fQ = fwd.Q_total - Qd;

            double obj = normalizedObj(fwd.T_total, fwd.Q_total, Td, Qd);

            if (obj < tol) {
                res.success   = true;
                res.omega_u   = std::clamp(u, coaxial_data::RPM_MIN,
                                              coaxial_data::RPM_MAX);
                res.omega_l   = std::clamp(l, coaxial_data::RPM_MIN,
                                              coaxial_data::RPM_MAX);
                res.T_pred    = fwd.T_total;
                res.Q_pred    = fwd.Q_total;
                res.objective = obj;
                res.status    = "converged";
                return res;
            }

            // Jacobian J = [[dT/du, dT/dl], [dQ/du, dQ/dl]]
            auto J = jacobian(u, l);
            double a11 = J[0][0], a12 = J[0][1];
            double a21 = J[1][0], a22 = J[1][1];

            double det = a11 * a22 - a12 * a21;
            if (std::abs(det) < 1e-14) break;  // singular → fall to pattern search

            // Newton step: [du, dl] = -J^{-1} * [fT, fQ]
            double du = -(fT * a22 - fQ * a12) / det;
            double dl = -(a11 * fQ - a21 * fT) / det;

            // Damped step with line search (backtracking)
            double alpha = 1.0;
            for (int ls = 0; ls < 10; ++ls) {
                double u_new = u + alpha * du;
                double l_new = l + alpha * dl;
                // Soft boundary: allow slight overshoot, clamp later
                auto fwd_new = query(u_new, l_new, false);
                double obj_new = normalizedObj(fwd_new.T_total, fwd_new.Q_total,
                                               Td, Qd);
                if (obj_new < obj) {
                    u = u_new;
                    l = l_new;
                    break;
                }
                alpha *= 0.5;
            }
        }

        // Did not converge — return best point found
        u = std::clamp(u, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
        l = std::clamp(l, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
        auto fwd = query(u, l);
        res.success   = false;
        res.omega_u   = u;
        res.omega_l   = l;
        res.T_pred    = fwd.T_total;
        res.Q_pred    = fwd.Q_total;
        res.objective = normalizedObj(fwd.T_total, fwd.Q_total, Td, Qd);
        return res;
    }

    // ---- Pattern search fallback (derivative-free) ----
    AeroInverseResult patternSearch(double Td, double Qd,
                                    double u0, double l0,
                                    double tol) const
    {
        double u = std::clamp(u0, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);
        double l = std::clamp(l0, coaxial_data::RPM_MIN, coaxial_data::RPM_MAX);

        auto fwd = query(u, l);
        double best_obj = normalizedObj(fwd.T_total, fwd.Q_total, Td, Qd);

        double step_u = 0.2 * (coaxial_data::RPM_MAX - coaxial_data::RPM_MIN);
        double step_l = step_u;

        for (int outer = 0; outer < 40; ++outer) {
            bool improved = false;

            for (int du = -1; du <= 1; ++du) {
                for (int dl = -1; dl <= 1; ++dl) {
                    double cu = std::clamp(u + du * step_u,
                                           coaxial_data::RPM_MIN,
                                           coaxial_data::RPM_MAX);
                    double cl = std::clamp(l + dl * step_l,
                                           coaxial_data::RPM_MIN,
                                           coaxial_data::RPM_MAX);
                    auto qr = query(cu, cl);
                    double obj = normalizedObj(qr.T_total, qr.Q_total, Td, Qd);
                    if (obj < best_obj) {
                        best_obj = obj;
                        u = cu;
                        l = cl;
                        fwd = qr;
                        improved = true;
                    }
                }
            }

            if (!improved) {
                step_u *= 0.5;
                step_l *= 0.5;
            }
            if (step_u < 0.5 && step_l < 0.5) break;
            if (best_obj < tol) break;
        }

        AeroInverseResult res;
        res.success   = (best_obj < 1e-4);
        res.omega_u   = u;
        res.omega_l   = l;
        res.T_pred    = fwd.T_total;
        res.Q_pred    = fwd.Q_total;
        res.objective = best_obj;
        res.status    = (best_obj < tol) ? "converged" : "pattern_search";
        return res;
    }
};
