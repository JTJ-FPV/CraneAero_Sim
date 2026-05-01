/**
 * test_allocator_qp.cpp
 *
 * Test the QP allocator across normal and pathological scenarios:
 *   1. Hover (baseline — should match analytic solution)
 *   2. Pure roll / pure pitch / pure yaw
 *   3. Saturation — request more thrust than achievable
 *   4. Conflicting requests — max thrust + max yaw simultaneously
 *
 * Build:
 *   g++ -std=c++17 -O2 -I/usr/include/eigen3 test_allocator_qp.cpp -losqp -o test_qp
 */
#include <cstdio>
#include <type_traits>
#include <Eigen/Dense>
#include "coaxial_rotor_model.hpp"
#include "coaxial_x8_allocator_qp.hpp"

static void printResult(const char* label, double F_des, const Eigen::Vector3d& tau_des,
                         const CoaxialX8AllocatorQP::Result& r)
{
    printf("\n=== %s ===\n", label);
    printf("Request : F=%.2f  Mx=%.3f  My=%.3f  Mz=%.3f\n",
           F_des, tau_des(0), tau_des(1), tau_des(2));
    printf("QP status: %s (iters=%d, residual=%.4f)\n",
           r.qp_success ? "SOLVED" : "FALLBACK",
           r.qp_iters, r.residual);
    printf("Achieved : F=%.2f  Mx=%.3f  My=%.3f  Mz=%.3f\n",
           r.wrench_ach(0), r.wrench_ach(1), r.wrench_ach(2), r.wrench_ach(3));
    printf("T_pair   : ");
    for (int p = 0; p < 4; ++p) printf("%7.2f ", r.T_pair(p));
    printf("\nQ_pair   : ");
    for (int p = 0; p < 4; ++p) printf("%7.2f ", r.Q_pair(p));
    printf("\nRPMs     :");
    for (int p = 0; p < 4; ++p)
        printf(" [u=%.0f, l=%.0f]", r.rpm(2*p), r.rpm(2*p+1));
    printf("\n");
}

int main()
{

    std::cout << "OSQP version = " << osqp_version() << "\n";
    std::cout << "sizeof(OSQPInt) = " << sizeof(OSQPInt) << "\n";
    std::cout << "sizeof(Eigen default sparse index) = "
              << sizeof(Eigen::SparseMatrix<double>::StorageIndex) << "\n";

    CoaxialX8AllocatorQP::Params ap;
    ap.arm_length = 1.65;
    ap.verbose    = false;
    CoaxialX8AllocatorQP alloc(ap);

    printf("Pair bounds derived from CFD data:\n");
    printf("  T ∈ [%.2f, %.2f] N\n", alloc.getTmin(), alloc.getTmax());
    printf("  Q ∈ [%.2f, %.2f] Nm\n", alloc.getQmin(), alloc.getQmax());

    // ---- Assumed vehicle: 25 kg, g=9.81 → hover weight ≈ 245 N ----
    const double g = 9.81;
    const double mass = 500.0;
    const double W = mass * g;

    // 1. Hover
    {
        std::cout << "Hover test: request " << W << " N thrust, zero moments" << std::endl;
        auto r = alloc.allocate(W, Eigen::Vector3d(0, 0, 0));
        printResult("1. Hover", W, Eigen::Vector3d(0, 0, 0), r);
    }

    // 2. Pure roll
    {
        Eigen::Vector3d tau(20.0, 0, 0);
        auto r = alloc.allocate(W, tau);
        printResult("2. Pure roll (20 Nm)", W, tau, r);
    }

    // 3. Pure pitch
    {
        Eigen::Vector3d tau(0, 20.0, 0);
        auto r = alloc.allocate(W, tau);
        printResult("3. Pure pitch (20 Nm)", W, tau, r);
    }

    // 4. Pure yaw
    {
        Eigen::Vector3d tau(0, 0, 20.0);
        auto r = alloc.allocate(W, tau);
        printResult("4. Pure yaw (20 Nm)", W, tau, r);
    }

    // 5. Combined maneuver
    {
        Eigen::Vector3d tau(15, 10, 8);
        auto r = alloc.allocate(W, tau);
        printResult("5. Combined (roll+pitch+yaw)", W, tau, r);
    }

    // 6. Saturation test — ask for unrealistic thrust
    {
        auto r = alloc.allocate(50000.0, Eigen::Vector3d(0, 0, 0));
        printResult("6. Over-thrust (50000 N, impossible)", 50000.0,
                    Eigen::Vector3d(0, 0, 0), r);
    }

    // 7. Conflicting — max thrust + max yaw simultaneously
    {
        Eigen::Vector3d tau(0, 0, 100.0);
        auto r = alloc.allocate(alloc.getTmax() * 4 * 0.9, tau);
        printResult("7. Near-max thrust + strong yaw", alloc.getTmax() * 4 * 0.9, tau, r);
    }

    // 8. Zero thrust (landing)
    {
        auto r = alloc.allocate(0.0, Eigen::Vector3d(0, 0, 0));
        printResult("8. Zero thrust (landing)", 0.0, Eigen::Vector3d(0, 0, 0), r);
    }

    return 0;
}