#include "aerodynamic.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    std::vector<Sample> data = {
        {  900.0,  900.0,  206.23853,   1.6241847 },
        { 1200.0, 1200.0,  369.48488,   2.9322684 },
        { 1500.0, 1500.0,  582.71436,   4.5915645 },
        { 1800.0, 1800.0,  845.48866,   6.8774154 },
        { 2000.0, 2000.0, 1050.97260,   8.5094328 },
        { 2245.0, 2245.0, 1333.55740,  11.1445840 },
        { 1250.0, 1150.0,  364.97592,   7.3475182 },
        { 1150.0, 1250.0,  378.31475,  -1.5272520 },
        { 1300.0, 1100.0,  363.60064,  11.7722270 },
        { 1100.0, 1300.0,  390.99528,  -5.9763659 },
        { 1900.0, 1700.0,  833.61766,  20.5140750 },
        { 1700.0, 1900.0,  874.03961,  -6.9758052 },
        { 2000.0, 1600.0,  833.39759,  34.5758130 },
        { 1600.0, 2000.0,  917.98961, -20.8093510 },
        { 2245.0, 2045.0, 1197.73030,  26.9304200 },
        { 2045.0, 2245.0, 1246.84370,  -7.0552301 },
        { 1100.0, 1100.0,  309.62666,   2.4535291 },
        { 1300.0, 1300.0,  435.22961,   3.4133697 },
        { 1600.0, 1600.0,  664.21206,   5.3497721 },
        { 1700.0, 1700.0,  752.62528,   5.9832646 },
        { 1900.0, 1900.0,  944.85962,   7.7215857 },
        { 2045.0, 2045.0, 1100.44690,   8.9353564 },
        { 2145.0, 2145.0, 1213.55960,  10.0645650 },
        { 2145.0, 2245.0, 1288.45420,   1.6752560 },
        { 2245.0, 2145.0, 1262.24150,  19.3455230 }
    };

    CoaxialPairInterpolator model(data);

    // 例子 1：用已知样本值来反解
    {
        double T_des = 944.85962;
        double Q_des = 7.7215857;

        auto inv = model.invert(T_des, Q_des, true);

        std::cout << "[Case 1]\n";
        std::cout << "success = " << inv.success << "\n";
        std::cout << "status  = " << inv.status << "\n";
        std::cout << "omega_u = " << inv.omega_u << "\n";
        std::cout << "omega_l = " << inv.omega_l << "\n";
        std::cout << "T_pred  = " << inv.T_pred << "\n";
        std::cout << "Q_pred  = " << inv.Q_pred << "\n";
        std::cout << "obj     = " << inv.objective << "\n\n";
    }

    // 例子 2：一个内部插值目标
    {
        double T_des = 813.0;
        double Q_des = -0.5;

        auto inv = model.invert(T_des, Q_des, true);

        std::cout << "[Case 2]\n";
        std::cout << "success = " << inv.success << "\n";
        std::cout << "status  = " << inv.status << "\n";
        std::cout << "omega_u = " << inv.omega_u << "\n";
        std::cout << "omega_l = " << inv.omega_l << "\n";
        std::cout << "T_pred  = " << inv.T_pred << "\n";
        std::cout << "Q_pred  = " << inv.Q_pred << "\n";
        std::cout << "obj     = " << inv.objective << "\n\n";
    }

    // 例子 3：目标可能超出当前可行域
    {
        double T_des = 1400.0;
        double Q_des = 0.0;

        auto inv = model.invert(T_des, Q_des, true);

        std::cout << "[Case 3]\n";
        std::cout << "success = " << inv.success << "\n";
        std::cout << "status  = " << inv.status << "\n";
        std::cout << "omega_u = " << inv.omega_u << "\n";
        std::cout << "omega_l = " << inv.omega_l << "\n";
        std::cout << "T_pred  = " << inv.T_pred << "\n";
        std::cout << "Q_pred  = " << inv.Q_pred << "\n";
        std::cout << "obj     = " << inv.objective << "\n\n";
    }

    return 0;
}