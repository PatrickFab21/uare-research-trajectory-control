# uare-research-trajectory-control

This repository is part of an academic research project focused on **trajectory control for UAVs (drones)** in simulated environments using **ROS 2** and **Gazebo Classic**. It builds upon the open-source [`sjtu_drone`](https://github.com/NovoG93/sjtu_drone) simulator and adds custom controllers, improved dynamics, and additional functionality for research in navigation and control.

---

## 📌 Overview

The goal of this project is to simulate, control, and evaluate various UAV trajectory tracking algorithms. Key contributions include:

- Altitude PID controller for real-time height regulation.
- Target point navigation using full 3D PID.
- Circular trajectory controller with multi-phase logic.
- Polygonal trajectory controller supporting smooth transitions.
- Modified launch and configuration files.
- Integration with ROS 2 topics for takeoff, landing, and velocity control.
- Realistic drone behavior via Gazebo with simulated sensors (IMU, GPS, camera, sonar).

---

## 🚀 Getting Started

### Requirements

- ROS 2 Humble (Ubuntu 22.04)
- Gazebo Classic (Gazebo 11)
- `sjtu_drone` simulator (required as base)

### Installation

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/uare-research-trajectory-control.git
```

2. Clone the required base simulator:

```bash
git clone https://github.com/NovoG93/sjtu_drone.git
```

3. Install dependencies and build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 🧪 Usage

To launch the simulation with your custom controller:

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

### Takeoff:
```bash
ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty '{}' --once
```

### Land:
```bash
ros2 topic pub /simple_drone/land std_msgs/msg/Empty '{}' --once
```

---

## 📊 Implemented Controllers

All control nodes are located in the folder: `sjtu_drone_control/sjtu_drone_control/`

### `altitude_controller.py` — Altitude Control
This node controls the drone's vertical position using a PID controller.

- 📍 Path: `sjtu_drone_control/sjtu_drone_control/altitude_controller.py`
- 📐 Parameters:
  - `target_altitude` (float, default: 10.0): desired flight altitude in meters.
- 🧠 It regulates vertical velocity `v_z` to reach and maintain `target_altitude`.

---

### `point_to_achieve_v1.py` — 3D Target PID Navigation
This node moves the UAV to a specific 3D target position using PID control over yaw, altitude, and distance.

- 📍 Path: `sjtu_drone_control/sjtu_drone_control/point_to_achieve_v1.py`
- 📐 Parameters:
  - Target position: `x`, `y`, `z`
  - PID gains: `k_yaw`, `k_dist`, `k_z`
  - Max velocities: `v_max`, `w_max`
  - Tolerances: `epsilon_yaw`, `epsilon_dist`
- 🧠 It aligns yaw before advancing, using real-time odometry from `/simple_drone/odom`.

---

### `circular_trajectory_PID_v2.py` — Circular Trajectory Tracking
Tracks a circular path by transitioning between approach and follow phases using PD control.

- 📍 Path: `sjtu_drone_control/sjtu_drone_control/circular_trajectory_PID_v2.py`
- 📐 Parameters:
  - Circle center: `X_C`, `Y_C`
  - Radius `R`, altitude `Z_D`
  - Gains: `K_p_xy`, `K_d_xy`, `K_p_z`, `K_d_z`
  - Tolerances: `epsilon_approach`, `epsilon_z`
- 🧠 After reaching the circle perimeter, the drone transitions to smooth angular motion along the path.

---

### `points_to_achieve_v1.py` — Polygonal Waypoint Navigation
Controls the UAV along a polygon (triangle, square, etc.) using sequential waypoint tracking.

- 📍 Path: `sjtu_drone_control/sjtu_drone_control/points_to_achieve_v1.py`
- 📐 Parameters:
  - `num_vertices` (int)
  - Gains: `k_psi`, `k_d`, `k_z`
  - Max velocities and waypoint threshold
- 🧠 Switches to the next waypoint once the UAV reaches the current target.

---

### `points_to_achieve_v3_smooth.py` — Smooth Polygonal Navigation
Enhances polygonal navigation with smoothed corner transitions using a radius-based switch.

- 📍 Path: `sjtu_drone_control/sjtu_drone_control/points_to_achieve_v3_smooth.py`
- 📐 Parameters:
  - `num_vertices`, `corner_radius`
  - Gains: `k_psi`, `k_d`, `k_z`
  - Velocity and angular rate limits
- 🧠 Preemptively switches to the next waypoint within a corner radius for smooth motion.

---

## 🙏 Acknowledgements

This project makes extensive use of the [`sjtu_drone`](https://github.com/NovoG93/sjtu_drone) simulator developed by [NovoG93](https://github.com/NovoG93) and contributors from Shanghai Jiao Tong University.\
We sincerely thank the authors for their work and for releasing it under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html), which made this research possible.

---

## 📄 License

This repository is distributed under the **GNU General Public License v3.0 (GPL-3.0)**.\
See the [LICENSE](LICENSE) file for more information.

