#pragma once
/**
 * fuselage_aero_model.hpp  —  Fuselage CFD-based aero model (UNIFIED frame)
 *
 * Inputs : V   = velocity along body-x [m/s] (signed)
 *          AoA = pitch angle of attack [deg]
 * Outputs: Force  (Fx, Fy, Fz) and Torque (Mx, My, Mz) in body frame.
 *
 * Method : Cubic RBF interpolation, φ(r) = r^3.
 *          Selected via benchmark (best LOO + best strict-extrapolation).
 *
 * Data   : 25 CFD samples on V ∈ {-10,-5,10,20,30}, AoA ∈ {-10,-5,0,5,10}.
 *          Coordinates already unified — DO NOT apply any sign correction.
 */
#include <eigen3/Eigen/Dense>
#include <array>
#include <cmath>

class FuselageAeroModel
{
public:
    struct Result {
        Eigen::Vector3d force;   // [Fx, Fy, Fz] in body frame  [N]
        Eigen::Vector3d torque;  // [Mx, My, Mz] in body frame  [N·m]
    };

    FuselageAeroModel() { build(); }

    Result query(double V, double AoA_deg) const
    {
        Eigen::Matrix<double, N_, 1> phi;
        for (int i = 0; i < N_; ++i) {
            const double dV = V        - DATA[i][0];
            const double dA = AoA_deg  - DATA[i][1];
            const double r  = std::sqrt(dV * dV + dA * dA);
            phi(i) = r * r * r;                       // cubic RBF
        }
        Eigen::Matrix<double, 6, 1> y = weights_.transpose() * phi;
        Result r;
        r.force  << y(0), y(1), y(2);
        r.torque << y(3), y(4), y(5);
        return r;
    }

    // Data hull — useful for clamping
    static constexpr double V_MIN   = -10.0, V_MAX   = 30.0;
    static constexpr double AOA_MIN = -10.0, AOA_MAX = 10.0;

private:
    static constexpr int N_ = 30;

    // [V, AoA, Fx, Fy, Fz, Mx, My, Mz]   — UNIFIED body frame
    static constexpr std::array<std::array<double,8>, N_> DATA = {{
        {-10,-10,  33.075552,  33.799540,  13.560373,   9.518495, -32.655919, -37.032031},
        {-10, -5,  31.600918,  29.469532,   9.219464,   6.097487, -32.022870, -34.032580},
        {-10,  0,  29.134575,  26.267403,   5.267684,   3.317098, -30.894647, -30.878125},
        {-10,  5,  31.265028,  26.019851,   1.656630,   1.441946, -34.575073, -31.328177},
        {-10, 10,  32.931987,  25.766898,  -3.423874,  -0.659705, -38.842004, -32.728608},
        { 10,-10, -32.704501,  28.616957,  -7.268653,  -7.252899,  15.722893,  19.159995},
        { 10, -5, -30.804716,  26.405881,  -2.265087,  -4.906666,  13.769902,  18.129849},
        { 10,  0, -28.614849,  24.707290,   2.044702,  -2.624938,  11.035542,  16.998473},
        { 10,  5, -30.555526,  26.473255,   5.360712,  -0.087288,   9.914613,  18.326233},
        { 10, 10, -32.088001,  29.027877,   9.446726,   2.654385,   8.952792,  18.952769},
        { 20,-10,-128.662292, 115.142823, -27.401178, -28.622403,  63.315831,  74.874138},
        { 20, -5,-121.380607, 106.413879,  -8.522947, -19.162097,  54.921672,  70.917978},
        { 20,  0,-112.614800,  99.892371,   7.934416, -10.404124,  43.913560,  66.790560},
        { 20,  5,-120.533439, 106.166823,  20.640214,  -0.874132,  38.705628,  72.078923},
        { 20, 10,-126.163392, 116.439783,  36.776488,  10.112165,  34.424131,  74.295929},
        { 30,-10,-288.693565, 259.988813, -59.952653, -64.246282, 143.361292, 167.649483},
        { 30, -5,-272.295079, 240.098523, -18.505543, -43.102312, 123.995869, 158.494772},
        { 30,  0,-252.271250, 225.801933,  17.319709, -23.499563,  98.243716, 149.729523},
        { 30,  5,-270.374393, 239.516085,  45.776127,  -2.439635,  86.528763, 161.627771},
        { 30, 10,-283.084244, 262.581409,  82.012122,  22.103169,  76.738580, 166.414372},
        { -5,-10,   8.4454,     8.3637,     3.4730,     2.3839,    -8.3303,    -9.3809},
        { -5, -5,   8.0434,     7.2757,     2.3309,     1.5121,    -8.1342,    -8.6123},
        { -5,  0,   6.5683,     6.5784,     1.2578,     0.70002,   -7.2117,    -7.2026},
        { -5,  5,   7.9452,     6.4432,     0.38995,    0.29817,   -8.7328,    -7.8950},
        { -5, 10,   8.4013,     6.3730,    -0.92437,   -0.23480,   -9.8386,    -8.2915},
        {  0,-10,        0,          0,           0,          0,         0,          0},
        {  0, -5,        0,          0,           0,          0,         0,          0},
        {  0,  0,        0,          0,           0,          0,         0,          0},
        {  0,  5,        0,          0,           0,          0,         0,          0},
        {  0, 10,        0,          0,           0,          0,         0,          0}
    }};

    Eigen::Matrix<double, N_, 6> weights_;

    void build()
    {
        // Φ_{ij} = ||x_i - x_j||^3   (symmetric, full)
        Eigen::Matrix<double, N_, N_> Phi;
        for (int i = 0; i < N_; ++i) {
            for (int j = 0; j < N_; ++j) {
                const double dV = DATA[i][0] - DATA[j][0];
                const double dA = DATA[i][1] - DATA[j][1];
                const double r  = std::sqrt(dV * dV + dA * dA);
                Phi(i, j) = r * r * r;
            }
        }
        Eigen::Matrix<double, N_, 6> Y;
        for (int i = 0; i < N_; ++i)
            for (int k = 0; k < 6; ++k)
                Y(i, k) = DATA[i][2 + k];

        // Solve Φ W = Y  (one factorization, 6 RHS)
        weights_ = Phi.colPivHouseholderQr().solve(Y);
    }
};