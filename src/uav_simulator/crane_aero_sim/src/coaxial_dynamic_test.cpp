#include <cstdio>
#include <cmath>
#include "dynamics.hpp"
#include "coaxial_rotor_model.hpp"
#include "coaxial_x8_dynamics.hpp"
#include "coaxial_x8_allocator.hpp"

int main(int argc, char** argv)
{
    // ============================================================
    // Vehicle parameters (adjust to your actual vehicle!)
    // ============================================================
    const double mass = 25.0;   // kg — placeholder, set to your vehicle
    const double g    = 9.81;

    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = 0.5;   // Ixx [kg·m²]
    I(1,1) = 0.5;   // Iyy
    I(2,2) = 0.8;   // Izz

    // ============================================================
    // Test 1: Hover
    // ============================================================
    printf("=== Test 1: Hover equilibrium ===\n");

    CoaxialX8Allocator::Params alloc_params;
    alloc_params.arm_length = 1.65;

    CoaxialX8Allocator allocator(alloc_params);
    auto hover_rpm = allocator.hoverRPM(mass * g);

    printf("Hover RPMs (weight=%.1f N):\n", mass * g);
    for (int p = 0; p < 4; ++p) {
        printf("  Pair %d: upper=%.1f, lower=%.1f RPM\n",
               p, hover_rpm(2*p), hover_rpm(2*p+1));
    }

    // Verify: query aero model for total thrust
    CoaxialRotorModel aero;
    double total_T = 0;
    for (int p = 0; p < 4; ++p) {
        auto q = aero.query(hover_rpm(2*p), hover_rpm(2*p+1));
        total_T += q.T_total;
    }
    printf("Total thrust at hover RPM: %.2f N  (target: %.2f N, err: %.2f%%)\n",
           total_T, mass*g, std::abs(total_T - mass*g)/(mass*g)*100);

    // ============================================================
    // Test 2: Dynamics step — drop from rest, check it accelerates down
    // ============================================================
    printf("\n=== Test 2: Free-fall sanity check (motors off) ===\n");

    coaxial_x8_dynamics sim(mass, I);
    sim.init(Eigen::Vector3d(0, 0, 10),
             Eigen::Vector4d(1, 0, 0, 0));

    // Zero RPM command — should fall
    Eigen::Matrix<double, 8, 1> zero_rpm = Eigen::Matrix<double, 8, 1>::Zero();
    // Set to minimum RPM (model clamps internally)
    zero_rpm.setConstant(CoaxialRotorModel::rpmMin());
    sim.setRPMCmd(zero_rpm);

    for (int i = 0; i < 100; ++i)
        sim.step_forward(0.01);

    auto pos = sim.getPos();
    auto vel = sim.getVel();
    printf("After 1.0s with min RPM: pos_z=%.3f m, vel_z=%.3f m/s\n",
           pos(2), vel(2));
    printf("Expected: pos_z < 10 (falling), vel_z < 0 (downward)\n");

    // ============================================================
    // Test 3: Hover simulation — RPMs from allocator
    // ============================================================
    printf("\n=== Test 3: Hover with allocator ===\n");

    coaxial_x8_dynamics sim2(mass, I);
    sim2.init(Eigen::Vector3d(0, 0, 10),
              Eigen::Vector4d(1, 0, 0, 0));

    sim2.setRPMCmd(hover_rpm);

    for (int i = 0; i < 500; ++i)
        sim2.step_forward(0.002);  // 500Hz, 1 second

    auto pos2 = sim2.getPos();
    auto vel2 = sim2.getVel();
    printf("After 1.0s hover: pos_z=%.4f m (target: 10.0), vel_z=%.4f m/s\n",
           pos2(2), vel2(2));
    printf("Attitude quaternion: [%.4f, %.4f, %.4f, %.4f]\n",
           sim2.getQuat()(0), sim2.getQuat()(1),
           sim2.getQuat()(2), sim2.getQuat()(3));

    // ============================================================
    // Test 4: Roll command
    // ============================================================
    printf("\n=== Test 4: Roll moment allocation ===\n");
    {
        Eigen::Vector3d tau(5.0, 0.0, 0.0);   // 5 Nm roll
        auto rpms = allocator.allocate(mass * g, tau);
        printf("Roll 5Nm RPMs:\n");
        for (int p = 0; p < 4; ++p) {
            printf("  Pair %d: upper=%.1f, lower=%.1f\n",
                   p, rpms(2*p), rpms(2*p+1));
        }
    }

    // ============================================================
    // Test 5: Yaw command
    // ============================================================
    printf("\n=== Test 5: Yaw moment allocation ===\n");
    {
        Eigen::Vector3d tau(0.0, 0.0, 3.0);   // 3 Nm yaw
        auto rpms = allocator.allocate(mass * g, tau);
        printf("Yaw 3Nm RPMs:\n");
        for (int p = 0; p < 4; ++p) {
            auto q = aero.query(rpms(2*p), rpms(2*p+1));
            printf("  Pair %d: upper=%.1f, lower=%.1f  → Q_total=%.3f Nm\n",
                   p, rpms(2*p), rpms(2*p+1), q.Q_total);
        }

        // Verify yaw torque
        double Mz = 0;
        for (int p = 0; p < 4; ++p) {
            auto q = aero.query(rpms(2*p), rpms(2*p+1));
            Mz += coaxial_x8_dynamics::PAIR_YAW_SIGN[p] * q.Q_total;
        }
        printf("Achieved Mz = %.3f Nm (target: 3.0 Nm)\n", Mz);
    }

    printf("\nAll integration tests complete.\n");
    return 0;
}