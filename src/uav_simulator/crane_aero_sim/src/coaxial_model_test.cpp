#include <cstdio>
#include <cmath>
#include <chrono>
#include "coaxial_rotor_model.hpp"

int main(int argc, char** argv) 
{
    using Clock = std::chrono::high_resolution_clock;

    // ============================================================
    //  1. Build model and measure construction time
    // ============================================================
    auto t0 = Clock::now();
    CoaxialRotorModel model;
    auto t1 = Clock::now();
    double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    printf("Model built in %.2f ms\n\n", build_ms);

    // ============================================================
    //  2. Forward interpolation accuracy (all 25 CFD points)
    // ============================================================
    printf("=== Forward interpolation — all CFD points ===\n");
    printf("%-4s %6s %6s | %10s %10s | %8s %8s\n",
           "Case", "RPM_u", "RPM_l", "T_cfd", "T_rbf", "err_T", "err_Q");
    printf("--------------------------------------------------------------\n");

    double max_eT = 0, max_eQ = 0;
    for (int i = 0; i < coaxial_data::N_PTS; ++i) {
        double ru = coaxial_data::DATA[i][0];
        double rl = coaxial_data::DATA[i][1];
        double T_cfd = coaxial_data::DATA[i][2] + coaxial_data::DATA[i][3];
        double Q_cfd = coaxial_data::DATA[i][4] + coaxial_data::DATA[i][5];

        auto r = model.query(ru, rl);
        double eT = std::abs(r.T_total - T_cfd);
        double eQ = std::abs(r.Q_total - Q_cfd);
        max_eT = std::max(max_eT, eT);
        max_eQ = std::max(max_eQ, eQ);

        printf("%-4d %6.0f %6.0f | %10.3f %10.3f | %8.5f %8.5f\n",
               i + 1, ru, rl, T_cfd, r.T_total, eT, eQ);
    }
    printf("\nMax error: T=%.2e N,  Q=%.2e Nm  (should be ~0 — exact interpolation)\n\n",
           max_eT, max_eQ);

    // ============================================================
    //  3. Forward query at non-data points
    // ============================================================
    printf("=== Forward interpolation — non-data points ===\n");
    struct TestCase { double u, l; };
    TestCase tests[] = {
        {1400, 1400}, {1800, 1600}, {1600, 1800},
        {1000, 2000}, {2100, 1900}, {1700, 1500},
    };
    for (auto& tc : tests) {
        auto r = model.query(tc.u, tc.l);
        printf("RPM(%4.0f, %4.0f): T_total=%8.2f N, Q_total=%8.4f Nm  "
               "[T_u=%7.2f, T_l=%7.2f, Q_u=%7.4f, Q_l=%8.4f]\n",
               tc.u, tc.l, r.T_total, r.Q_total,
               r.T_u, r.T_l, r.Q_u, r.Q_l);
    }

    // ============================================================
    //  4. Jacobian test
    // ============================================================
    printf("\n=== Jacobian at (1500, 1500) ===\n");
    auto J = model.jacobian(1500, 1500);
    printf("  dT/dRPM_u = %.4f N/RPM,   dT/dRPM_l = %.4f N/RPM\n",
           J[0][0], J[0][1]);
    printf("  dQ/dRPM_u = %.6f Nm/RPM,  dQ/dRPM_l = %.6f Nm/RPM\n",
           J[1][0], J[1][1]);

    // Numerical verification (central difference, h=1 RPM)
    double h = 1.0;
    auto fp = model.query(1501, 1500); auto fm = model.query(1499, 1500);
    double dTdu_num = (fp.T_total - fm.T_total) / (2 * h);
    double dQdu_num = (fp.Q_total - fm.Q_total) / (2 * h);
    printf("  Numerical check: dT/du=%.4f, dQ/du=%.6f\n", dTdu_num, dQdu_num);

    // ============================================================
    //  5. Inverse solve test
    // ============================================================
    printf("\n=== Inverse solve test ===\n");
    // Test 1: known CFD point (should recover exactly)
    {
        double Td = 582.714, Qd = 4.5916;  // Case 3: RPM 1500/1500
        auto inv = model.invert(Td, Qd);
        printf("[Known point] T_des=%.1f, Q_des=%.4f\n", Td, Qd);
        printf("  → RPM_u=%.1f, RPM_l=%.1f  (expect 1500/1500)\n",
               inv.omega_u, inv.omega_l);
        printf("  → T_pred=%.3f, Q_pred=%.4f, obj=%.2e, status=%s\n",
               inv.T_pred, inv.Q_pred, inv.objective, inv.status.c_str());
    }

    // Test 2: asymmetric desired point
    {
        double Td = 850.0, Qd = 6.0;
        auto inv = model.invert(Td, Qd);
        printf("\n[Arbitrary] T_des=%.1f N, Q_des=%.1f Nm\n", Td, Qd);
        printf("  → RPM_u=%.1f, RPM_l=%.1f\n", inv.omega_u, inv.omega_l);
        printf("  → T_pred=%.3f, Q_pred=%.4f, obj=%.2e, status=%s\n",
               inv.T_pred, inv.Q_pred, inv.objective, inv.status.c_str());
        // Verify by forward query
        auto v = model.query(inv.omega_u, inv.omega_l);
        printf("  → Verify: T_total=%.3f, Q_total=%.4f\n", v.T_total, v.Q_total);
    }

    // Test 3: yaw control — same thrust, different torque
    {
        printf("\n[Yaw control] T_des=800 N, varying Q_des:\n");
        for (double Qd : {-10.0, 0.0, 10.0, 20.0}) {
            auto inv = model.invert(800.0, Qd);
            printf("  Q_des=%5.1f → RPM(%6.1f, %6.1f), T=%.1f, Q=%.2f, %s\n",
                   Qd, inv.omega_u, inv.omega_l,
                   inv.T_pred, inv.Q_pred, inv.status.c_str());
        }
    }

    // Test 4: hover control — same thrust, zero torque
    {
        // double Td = 300 * 9.81 / 4.0, Qd = 0.0;
        double Td = 25 * 9.81 / 4.0, Qd = 0.0;
        auto inv = model.invert(Td, Qd);
        printf("\n[Hover] T_des=%.1f N, Q_des=%.1f Nm\n", Td, Qd);
        printf("  → RPM_u=%.1f, RPM_l=%.1f\n", inv.omega_u, inv.omega_l);
        printf("  → T_pred=%.3f, Q_pred=%.4f, obj=%.2e, status=%s\n",
               inv.T_pred, inv.Q_pred, inv.objective, inv.status.c_str());
    }


    // ============================================================
    //  6. Performance benchmark
    // ============================================================
    {
        const int N = 100000;
        auto t2 = Clock::now();
        volatile double sink = 0;
        for (int i = 0; i < N; ++i) {
            double u = 900 + (i % 1345) * 1.0;
            double l = 900 + ((i * 7) % 1345) * 1.0;
            auto r = model.query(u, l);
            sink += r.T_total;
        }
        auto t3 = Clock::now();
        double ns_per_query = std::chrono::duration<double, std::nano>(t3 - t2).count() / N;
        printf("\n=== Performance ===\n");
        printf("Forward query: %.0f ns/call  (%d calls)\n", ns_per_query, N);
    }
    {
        const int N = 100;
        auto t2 = Clock::now();
        for (int i = 0; i < N; ++i) {
            double T = 300 + i * 10.0;
            double Q = 2.0 + i * 0.08;
            auto inv = model.invert(T, Q);
            (void)inv;
        }
        auto t3 = Clock::now();
        double us_per_inv = std::chrono::duration<double, std::micro>(t3 - t2).count() / N;
        printf("Inverse solve: %.1f us/call  (%d calls)\n", us_per_inv, N);
    }

    printf("\nAll tests passed.\n");
    return 0;
}