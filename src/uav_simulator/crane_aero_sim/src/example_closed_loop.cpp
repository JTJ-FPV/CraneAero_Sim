/**
 * example_closed_loop.cpp
 *
 * Minimal example showing the complete control stack:
 *
 *   setpoint ──► CoaxialX8Controller ──► (F_body_z, τ_body)
 *                                                │
 *                                                ▼
 *                                    CoaxialX8Allocator ──► rpm[8]
 *                                                │
 *                                                ▼
 *                                    coaxial_x8_dynamics ──► (pos, vel, q, ω)
 *                                                │
 *                                                └─── feedback loop
 *
 * Build (needs Eigen3):
 *   g++ -std=c++17 -O2 -I/usr/include/eigen3 example_closed_loop.cpp -o demo
 */
#include <cstdio>
#include <eigen3/Eigen/Dense>

#include "dynamics.hpp"
#include "coaxial_rotor_model.hpp"
#include "coaxial_x8_dynamics.hpp"
#include "coaxial_x8_allocator.hpp"
#include "coaxial_x8_allocator_qp.hpp"
#include "coaxial_x8_controller.hpp"

int main()
{
    // ------------- Vehicle parameters -------------
    const double mass = 500.0;
    const double g    = 9.81;

    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0,0) = 0.5; I(1,1) = 0.5; I(2,2) = 0.8;

    // ------------- Controller -------------
    CoaxialX8Controller ctrl(500.0);   // 500 Hz

    CoaxialX8Controller::Params cp;
    cp.mass    = mass;
    cp.inertia = I;
    cp.g       = g;
    ctrl.setParams(cp);

    CoaxialX8Controller::Gains cg;
    cg.pos_stable_time = 3.0;
    cg.alt_stable_time = 2.0;
    cg.att_stable_time = 0.4;
    ctrl.setGains(cg);

    CoaxialX8Controller::Limits cl;
    cl.max_thrust_N     = 1.06 * mass * g;  // plenty of headroom
    cl.max_torque_xy_Nm = 40.0;
    cl.max_torque_z_Nm  = 40.0;
    ctrl.setLimits(cl);

    // ------------- Allocator -------------
    CoaxialX8Allocator::Params ap;
    ap.arm_length = 1.65;
    CoaxialX8Allocator allocator(ap);
    CoaxialX8AllocatorQP::Params ap_qp;
    ap_qp.arm_length = 1.65;
    ap_qp.verbose    = false;
    CoaxialX8AllocatorQP alloc(ap_qp);

    // ------------- Dynamics -------------
    coaxial_x8_dynamics sim(mass, I);
    sim.setGeometry(1.65, 0.40);
    sim.init(Eigen::Vector3d(0, 0, 0),
             Eigen::Vector4d(1, 0, 0, 0));

    // ------------- Mission: hover at (0,0,10), then step to (2,1,12) ---------
    auto setpoint = [](double t) {
        Eigen::Vector3d pos(0, 0, 10);
        if (t > 5.0) pos = Eigen::Vector3d(2.0, 1.0, 12.0);
        return pos;
    };

    const double dt        = 1.0 / 500.0;
    const double T_total   = 20.0;
    const int    N_steps   = static_cast<int>(T_total / dt);

    printf("%6s | %7s %7s %7s | %7s %7s %7s | %8s %8s %8s %8s\n",
           "t[s]", "x", "y", "z", "vx", "vy", "vz",
           "F_body", "Mx", "My", "Mz");

    for (int k = 0; k < N_steps; ++k) {
        const double t = k * dt;

        // ---- Feedback ----
        Eigen::Vector4d q = sim.getQuat();  // (w,x,y,z)
        Eigen::Quaterniond q_wb(q(0), q(1), q(2), q(3));

        ctrl.setFeedback(sim.getPos(), sim.getVel(),
                         q_wb, sim.getAngularVelBody());

        ctrl.setSetpoint(setpoint(t),
                         Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::Zero(),
                         0.0);

        // ---- Controller → wrench ----
        auto wrench = ctrl.run();

        // ---- Allocator → 8 RPMs ----
        // auto rpm8 = allocator.allocate(wrench.thrust_body_z, wrench.torque_body);
        // sim.setRPMCmd(rpm8);
        auto rpm8 = alloc.allocate(wrench.thrust_body_z, wrench.torque_body);
        sim.setRPMCmd(rpm8.rpm);

        // ---- Dynamics step ----
        sim.step_forward(dt);

        // ---- Log every 250 ms ----
        if (k % 125 == 0) {
            Eigen::Vector3d p = sim.getPos(), v = sim.getVel();
            printf("%6.2f | %7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f | "
                   "%8.2f %8.3f %8.3f %8.3f\n",
                   t, p.x(), p.y(), p.z(), v.x(), v.y(), v.z(),
                   wrench.thrust_body_z,
                   wrench.torque_body(0),
                   wrench.torque_body(1),
                   wrench.torque_body(2));
        }
    }
    return 0;
}