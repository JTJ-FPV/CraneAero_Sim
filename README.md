# Crane Aero Sim + EGO-Planner

A ROS-based **coaxial X8 multirotor simulator** (`crane_aero_sim`) for a heavy-lift crane-class aerial vehicle, integrated with the **EGO-Planner** local planner for autonomous obstacle-avoidance flight in cluttered environments.

This repository extends the original [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) framework by replacing the lightweight quadrotor model with a high-fidelity coaxial X8 platform whose aerodynamics are driven by **CFD-fitted RBF interpolators** for both the rotor pairs and the fuselage. A cascade SO(3) controller plus an **OSQP-based control allocator** closes the loop and feeds RPM commands to the simulated dynamics, while EGO-Planner provides smooth, ESDF-free local trajectories from a depth camera or dual Livox MID-360 LiDARs.

---

## 1. Overview

The system is a complete planning + control + simulation stack:

```
2D Nav Goal / Waypoints
        │
        ▼
  EGO-Planner  ───────►  traj_server  ───────►  /coaxial_dynamics_1/planning_cmd
                                                          │
                                                          ▼
                                             ┌─────────────────────────┐
                                             │   crane_control_node    │
                                             │ ┌─────────────────────┐ │
                                             │ │ CoaxialX8Controller │ │  Position PD
                                             │ │     (cascade)       │ │  + SO(3) attitude
                                             │ └──────────┬──────────┘ │
                                             │            ▼            │
                                             │ ┌─────────────────────┐ │
                                             │ │  CoaxialX8AllocQP   │ │  OSQP wrench → RPM
                                             │ │  (CFD-aware)        │ │
                                             │ └──────────┬──────────┘ │
                                             │            ▼            │
                                             │ ┌─────────────────────┐ │
                                             │ │ coaxial_x8_dynamics │ │  CFD-RBF rotor
                                             │ │  + fuselage aero    │ │  + fuselage aero
                                             │ └──────────┬──────────┘ │
                                             └────────────┼────────────┘
                                                          │
                                  ┌───────────────────────┴───────────────────┐
                                  ▼                                           ▼
                       /coaxial_1/simulator/odom                  Sensor simulation
                           (feedback to EGO)                  (depth camera / dual MID-360)
```

### Key features

- **CFD-driven coaxial rotor model** — A `CoaxialRotorModel` fits four 2-D cubic-RBF interpolators (one each for upper/lower thrust and torque) to 41 CFD sample points. Forward query and Jacobian-based inversion `(T_des, Q_des) → (RPM_u, RPM_l)` are both supported.
- **CFD fuselage aerodynamics** — A `FuselageAeroModel` reproduces the 6-DOF aero wrench on the airframe from `(V, AoA)` using a 25-sample CFD dataset and cubic RBF interpolation.
- **Geometric SO(3) cascade controller** — Position PD outer loop + SO(3) attitude inner loop (Lee et al. 2010), no Euler-angle singularities, outputs body-frame `(F_z, τ_xyz)` wrench.
- **OSQP control allocator** — Solves a weighted least-squares QP over `[T_pair_0..3, Q_pair_0..3]` with box constraints from the CFD data hull, then inverts the rotor model per-pair to recover `RPM_u, RPM_l`. Priority weighting `thrust > roll/pitch > yaw` is configurable.
- **X8 layout** — 4 arms × (upper + lower) coaxial pairs, 8 rotors total. Spin convention `{+1,-1, -1,+1, +1,-1, -1,+1}` so each pair can independently produce yaw torque.
- **Two sensor configurations**:
  - Depth camera (RealSense-style 640×480, 30 Hz) — `local_sensing_node`.
  - Dual Livox MID-360 LiDAR (top + bottom-facing, 360° × 90° FoV, 10 Hz, GPU ray-casting) — `local_sensing_lidar`, **modified from [MARSIM](https://github.com/hku-mars/MARSIM)** to support a dual-LiDAR mount (`opengl_render_node_dual_mid360_gpu`) with configurable per-sensor pose and to interface with the X8 odometry stream.
- **EGO-Planner integration** — ESDF-free gradient-based local replanning at >100 Hz with B-spline smoothing.
- **RViz visualization** — Anti-flicker `MarkerArray` rendering of the airframe + 8 propellers loaded from STL meshes, with FPV-camera support and live trajectory history.
- **CSV logging** — `crane_control_log_node` records the full control state (setpoint, pose, twist, wrench, RPM, allocator residual, QP iterations) at every simulation tick for post-flight analysis.

---

## 2. Repository Layout

This repo is meant to be cloned **directly into a catkin workspace's `src/` folder** (the repo root *is* the workspace `src/`):

```
<your_catkin_ws>/
└── src/                                 ← clone this repo here
    ├── README.md
    ├── .gitignore
    ├── planner/                         # EGO-Planner stack
    │   ├── bspline_opt/
    │   ├── path_searching/
    │   ├── plan_env/
    │   ├── plan_manage/
    │   │   └── launch/
    │   │       ├── run_in_sim_crane_camera.launch        # Camera-based EGO + crane sim
    │   │       ├── run_in_sim_crane_lidar.launch         # Dual-MID360 LiDAR EGO + crane sim
    │   │       ├── run_in_sim_crane_lidar_log.launch     # LiDAR + CSV-logging variant
    │   │       ├── advanced_param_crane_camera.xml
    │   │       ├── advanced_param_crane_lidar.xml
    │   │       ├── simulator_crane_camera.xml
    │   │       ├── simulator_crane_lidar.xml
    │   │       └── simulator_crane_lidar_log.xml
    │   └── traj_utils/
    └── uav_simulator/
        ├── crane_aero_sim/              # ★ Custom coaxial X8 simulator
        │   ├── include/
        │   │   ├── dynamics.hpp                   # 6-DOF rigid-body integrator
        │   │   ├── coaxial_x8_dynamics.hpp        # X8 plant (rotor + fuselage aero)
        │   │   ├── coaxial_rotor_model.hpp        # CFD-fit cubic-RBF rotor model
        │   │   ├── fuselage_aero_model.hpp        # CFD-fit fuselage wrench
        │   │   ├── coaxial_x8_controller.hpp      # Cascade pos + SO(3) attitude
        │   │   ├── coaxial_x8_allocator.hpp       # Closed-form CFD allocator
        │   │   ├── coaxial_x8_allocator_qp.hpp    # OSQP-based allocator
        │   │   ├── visualizer.hpp                 # RViz MarkerArray renderer
        │   │   └── aerodynamic.hpp / delaunator.hpp
        │   ├── src/
        │   │   ├── crane_control_node.cpp         # Main control + dynamics node
        │   │   ├── crane_control_log_node.cpp     # Same + CSV logging
        │   │   ├── visualizer_node.cpp            # RViz visual node
        │   │   ├── example_closed_loop.cpp        # Standalone demo
        │   │   ├── coaxial_model_test.cpp
        │   │   ├── coaxial_dynamic_test.cpp
        │   │   ├── aerodynamic_test.cpp
        │   │   ├── test_allocator_qp.cpp
        │   │   └── test_so3_mass_stability.cpp
        │   ├── launch/
        │   │   ├── crane_camera.launch
        │   │   ├── crane_mid360.launch
        │   │   └── crane_control_node.launch
        │   ├── config/
        │   │   ├── visualizer_config.yaml         # Mesh paths, propeller layout
        │   │   ├── ego_visual.rviz
        │   │   ├── ego_lidar.rviz
        │   │   └── ego_lidar_fpv.rviz
        │   ├── meshes/CraneAero/                  # body.stl, propeller_cw/ccw.stl
        │   └── data/
        │       └── cfd_data.txt                   # 41-pt coaxial CFD dataset
        ├── local_sensing/                # Depth-camera renderer (CPU/CUDA)
        ├── local_sensing_lidar/          # Dual Livox MID-360 GPU renderer (modified from MARSIM)
        ├── map_generator/                # Random forest + .pcd map publisher
        ├── mockamap/                     # Perlin-noise map generator
        ├── so3_quadrotor_simulator/      # (legacy quadrotor — unused)
        └── Utils/                        # odom_visualization, quadrotor_msgs, etc.
```

---

## 3. Vehicle Specifications

| Parameter            | Value                | Notes                                                           |
|----------------------|----------------------|-----------------------------------------------------------------|
| Configuration        | Coaxial X8           | 4 arms × (upper + lower)                                        |
| Mass (default)       | 400 – 460 kg          | Heavy-lift "crane" class — set via `mass` ROS param             |
| Arm length           | 1.65 m                | Rotor-hub to body-z axis                                        |
| Rotor vertical sep.  | 0.40 m                | Upper / lower hub spacing                                       |
| RPM range            | ~900 – 2245 RPM       | Hull of the CFD dataset                                         |
| Max single-rotor T   | ~811 N (CFD case 6)   | At RPM = 2245                                                   |
| Max single-pair T    | ~1333 N               | At RPM_u = RPM_l = 2245                                         |
| Inertia model        | Mass-scaled           | `J = m · diag(KJX, KJY, KJZ)` from disk + central-body geometry |

The default `mass = 460` reflects the target heavy-lift platform; other values still fly correctly because the controller and allocator are reparameterized at runtime.

---

## 4. Dependencies

Tested on **Ubuntu 18.04 / 20.04** with **ROS Melodic / Noetic** (desktop-full). CUDA is required only if the LiDAR sensor simulation is enabled.

```bash
# Core ROS bits
sudo apt-get install libarmadillo-dev libeigen3-dev

# RViz components and GLFW3 development/runtime libraries
sudo apt install 'ros-noetic-rviz*' libglfw3-dev libglfw3

# OSQP — required by the QP-based control allocator (latest version from master)
git clone --recursive https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --target install
sudo ldconfig
```

> **OSQP version**: this project tracks the **latest** OSQP release (v1.x master). The QP allocator (`coaxial_x8_allocator_qp.hpp`) uses the current C API (`OSQPSolver`, `osqp_setup`, `osqp_solve`). If you have an older 0.6.x install on your system, please remove or upgrade it first to avoid header / symbol mismatches.

LiDAR sensing additionally needs an OpenGL / GPU-capable driver (the `local_sensing_lidar` package uses `opengl_render_node_dual_mid360_gpu`).

---

## 5. Build

This repo is the `src/` of a catkin workspace, so create a workspace folder, clone into its `src`, then build from the workspace root:

```bash
# 1. Create a catkin workspace
mkdir -p ~/crane_ws/src
cd ~/crane_ws/src

# 2. Clone this repo INTO the src folder (the trailing dot matters)
git clone <your-repo-url> .

# 3. Build from the workspace root
cd ~/crane_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

For VS Code IntelliSense, also pass `-DCMAKE_EXPORT_COMPILE_COMMANDS=Yes`.

> **Note on GPU LiDAR**: if you do not have CUDA / OpenGL set up, comment out the `local_sensing_lidar` related nodes in `simulator_crane_lidar.xml` and use the camera launch instead.

---

## 6. Quick Start

Source the workspace in **every terminal you open**:

```bash
source devel/setup.bash
```

### 6.1 EGO-Planner with depth camera (recommended first run)

```bash
roslaunch ego_planner run_in_sim_crane_camera.launch
```

This brings up:
- `random_forest` map generator (200 cylinders + 400 circles in a 100×100×30 m volume)
- `crane_control_node` (mass = 200 kg in the camera launch by default)
- `visualizer_node` (RViz markers for body + propellers)
- `pcl_render_node` (depth camera, 640×480 @ 30 Hz)
- `ego_planner_node` + `traj_server` + `waypoint_generator`
- RViz with `ego_visual.rviz`

In RViz, click **2D Nav Goal** and pick a target — EGO-Planner replans and the crane flies the trajectory.

### 6.2 EGO-Planner with dual MID-360 LiDARs

```bash
roslaunch ego_planner run_in_sim_crane_lidar.launch
```

This loads `map2.pcd` from `map_generator/resource/`, simulates a top-mounted and a bottom-mounted Livox MID-360 (both 360° × 90° FoV) via GPU ray-casting, and opens two RViz windows: a top-down view and an FPV view from the vehicle.

### 6.3 LiDAR variant with CSV control logging

```bash
roslaunch ego_planner run_in_sim_crane_lidar_log.launch
```

Identical to 6.2 but uses `crane_control_log_node`, which streams the full control state (39 columns: time, mode, setpoint, pose, twist, quaternion, body rates, thrust/torque commands, all 8 RPMs, allocator residual, QP iteration count) to `data/crane_log_<timestamp>.csv` for offline analysis.

### 6.4 Standalone simulator (no planner)

To bring up just the dynamics + visualizer without EGO-Planner:

```bash
roslaunch crane_aero_sim crane_camera.launch     # depth camera
# or
roslaunch crane_aero_sim crane_mid360.launch     # dual MID-360
```

You can publish `geometry_msgs/PoseStamped` to `/coaxial_dynamics_1/cmd_pose` to drive the vehicle directly.

### 6.5 RC / joystick control

The control node listens on `/joy` and switches modes on **button 8** (rising edge):

```
RC_CONTROL  ──btn8─▶  CMD_WAITING  ──first cmd_pose / planning_cmd──▶  POSITION / PLANNING
       ▲                                                                       │
       └──────────────────────── btn8 (any indexed mode → RC) ─────────────────┘
```

When entering `CMD_WAITING`, the setpoint is snapped to the current pose so the vehicle holds position until a real command arrives.

---

## 7. ROS Interface

### Topics (in `crane_control_node` namespace `coaxial_dynamics_<id>`)

| Topic                                | Direction | Type                              | Purpose                                       |
|--------------------------------------|-----------|-----------------------------------|-----------------------------------------------|
| `~odom`  → `/coaxial_<id>/simulator/odom` | Pub       | `nav_msgs/Odometry`               | True 6-DOF state (used as VIO/LIO surrogate)  |
| `~imu`                                | Pub       | `sensor_msgs/Imu`                 | Body angular velocity + linear acceleration   |
| `~cmd_rpm`                            | Pub       | `std_msgs/Float32MultiArray`      | 8-element commanded RPM vector                |
| `~cmd_pose`                           | Sub       | `geometry_msgs/PoseStamped`       | Manual position + yaw setpoint                |
| `~planning_cmd`                       | Sub       | `quadrotor_msgs/PositionCommand`  | EGO-Planner trajectory point (pos+vel+acc+yaw) |
| `/joy`                                | Sub       | `sensor_msgs/Joy`                 | RC input + mode toggle                        |

### Visualizer (`visualizer_node`)

Publishes one `visualization_msgs/MarkerArray` on `vehicle_markers` (anti-flicker — single atomic update per frame). Path history is published separately at 5 Hz, capped at 4000 poses.

---

## 8. Configuration

### `config/visualizer_config.yaml`

Defines mesh paths and the body-frame layout of the 8 propellers (4 pairs × upper/lower). Each propeller has an `origin` (hub position) and an `axis` (rotation axis, auto-normalized) so out-of-plane rotor cants are trivial to model.

### Tuning the controller

In `crane_control_node.cpp`, the cascade gains are expressed in terms of *settling time* and *damping ratio* rather than raw `Kp/Kd` — this is closer to physical intuition:

```cpp
cg.pos_stable_time = 3.2;   // s — outer-loop XY response
cg.alt_stable_time = 3.8;   // s — outer-loop Z response
cg.att_stable_time = 0.9;   // s — inner-loop attitude response
cg.att_damping     = 0.9;
```

The QP allocator weights default to `w_thrust = 100, w_moment = 10, w_yaw = 1`, prioritizing altitude tracking over yaw authority — important for a heavy-lift platform where yaw is naturally underactuated by the coaxial geometry.

---

## 9. Standalone Tests

Five lightweight test executables live under `crane_aero_sim/src/`:

| Executable             | What it does                                                             |
|------------------------|--------------------------------------------------------------------------|
| `aerodynamic_test`     | Sanity-check the legacy Delaunay-triangulation aerodynamic interpolator. |
| `coaxial_model_test`   | Forward-query and Jacobian sweep of the CFD rotor RBF model.             |
| `coaxial_dynamic_test` | Open-loop step input to `coaxial_x8_dynamics`.                           |
| `test_allocator_qp`    | OSQP allocator unit test with a battery of wrench requests.              |
| `example_closed_loop`  | Self-contained closed-loop demo (no ROS required to follow the logic).   |
| `test_so3_mass_stability` | Stress-test the SO(3) controller across a wide mass range.            |

Run them with `rosrun crane_aero_sim <executable_name>` after building.

---

## 10. Acknowledgements

This project builds on, and gratefully reuses code from, the following open-source efforts:

- **[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)** — *Zhou et al., RA-L 2020.* The ESDF-free gradient-based local planner that does the heavy lifting on trajectory generation.
- **[MARSIM](https://github.com/hku-mars/MARSIM)** — *Kong et al., RA-L 2023.* The point-realistic LiDAR simulator from HKU MaRS Lab. Our `local_sensing_lidar` package is **modified from MARSIM** to render a dual Livox MID-360 mount (top + bottom-facing) on the coaxial X8 platform.
- **[Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)** — Provided the original framework that EGO-Planner extends.
- **[mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap)** — The Perlin-noise random map generator.
- **[LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite)** — Header-only L-BFGS solver used inside EGO-Planner.
- **[OSQP](https://osqp.org/)** — The operator-splitting QP solver behind the control allocator.
- **[delaunator-cpp](https://github.com/abellgithub/delaunator-cpp)** — Bundled in `include/delaunator.hpp` for the legacy aerodynamic interpolator.

Citation for EGO-Planner:

> X. Zhou, Z. Wang, C. Xu, F. Gao, *EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors*, IEEE RA-L, 2020. [arXiv:2008.08835](https://arxiv.org/abs/2008.08835)

Citation for MARSIM:

> F. Kong, X. Liu, B. Tang, J. Lin, Y. Ren, Y. Cai, F. Zhu, N. Chen, F. Zhang, *MARSIM: A Light-weight Point-realistic Simulator for LiDAR-based UAVs*, IEEE RA-L, 2023. [arXiv:2211.10716](https://arxiv.org/abs/2211.10716)

---

## 11. License

The source code in this repository — including the `crane_aero_sim` simulator — is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html), consistent with the upstream EGO-Planner project.

---

## 12. Status

This is research-grade code. The CFD dataset is small (41 coaxial samples, 25 fuselage samples) and the RBF fits should not be extrapolated far outside their hull. The fuselage model assumes purely longitudinal flow (`V` along body-x, AoA on pitch); side-slip effects are not modeled. PRs and issues are welcome.
