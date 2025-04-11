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
git clone https://github.com/PatrickFab21/uare-research-trajectory-control.git
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

### Launch the Drone Simulation

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

### Basic Commands

- **Takeoff**:
  ```bash
  ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty '{}' --once
  ```

- **Land**:
  ```bash
  ros2 topic pub /simple_drone/land std_msgs/msg/Empty '{}' --once
  ```

---

## 📊 Implemented Controllers

All controller scripts are located under: `sjtu_drone_control/sjtu_drone_control/`

To run each controller, open a new terminal (with your ROS 2 environment sourced) and execute the corresponding `ros2 run` command.

### 1. Altitude Control
**File**: [Altitude Control Node](sjtu_drone_control/sjtu_drone_control/altitude_controller.py)

- **Basic Run**:
  ```bash
  ros2 run sjtu_drone_control altitude_controller
  ```
- **Run with Custom Parameter** (e.g., setting altitude to 15m):
  ```bash
  ros2 run sjtu_drone_control altitude_controller --ros-args -p target_altitude:=15.0
  ```
  This node keeps the drone at a desired altitude.

### 2. 3D Target PID Navigation
**File**: [Target PID Navigation Node](sjtu_drone_control/sjtu_drone_control/point_to_achieve_v1.py)

- **Basic Run**:
  ```bash
  ros2 run sjtu_drone_control point_to_achieve_v1
  ```
- **Run with Custom Parameters** (e.g., target (35,20,25)):
  ```bash
  ros2 run sjtu_drone_control point_to_achieve_v1 --ros-args \
    -p target_x:=35.0 -p target_y:=20.0 -p target_z:=25.0
  ```
  This node guides the drone to a specified 3D target position.

### 3. Circular Trajectory PID
**File**: [Circular Trajectory PID Control Node](sjtu_drone_control/sjtu_drone_control/circular_trajectory_PID_v2.py)

- **Basic Run**:
  ```bash
  ros2 run sjtu_drone_control circular_trajectory_PID_v2
  ```
- **Run with Custom Parameters**:
  ```bash
  ros2 run sjtu_drone_control circular_trajectory_PID_v2 --ros-args \
    -p approach_velocity:=10.0 \
    -p circle_velocity:=5.817 \
    -p circle_radius:=12.036
  ```
  This node tracks a circular path using a two-phase approach.

### 4. Polygonal Waypoint Navigation
**File**: [Polygonal Waypoint Navigation Node](sjtu_drone_control/sjtu_drone_control/points_to_achieve_v1.py)

- **Basic Run**:
  ```bash
  ros2 run sjtu_drone_control points_to_achieve_v1
  ```
- **Select a Polygon** (e.g., 4 vertices):
  ```bash
  ros2 run sjtu_drone_control points_to_achieve_v1 --ros-args -p num_vertices:=4
  ```
  This node follows a list of waypoints forming a polygon (3 to 7 vertices).

### 5. Smooth Polygonal Navigation
**File**: [Smooth Polygonal Navigation Node](sjtu_drone_control/sjtu_drone_control/points_to_achieve_v3_smooth.py)

- **Basic Run**:
  ```bash
  ros2 run sjtu_drone_control points_to_achieve_v3_smooth
  ```
- **Select a Polygon** (e.g., 5 vertices):
  ```bash
  ros2 run sjtu_drone_control points_to_achieve_v3_smooth --ros-args -p num_vertices:=5
  ```
  This node ensures smooth corner transitions along polygonal paths.

---

## 📈 MATLAB Codes

The `Matlab Codes/` folder contains energy-efficient trajectory optimization algorithms developed during the research phase. These include:

### 🔍 Circular Trajectory Optimization
**File**: [Circular Trajectory Optimization Matlab](Matlab Codes/UAV_3D_minEnergy_plusMinVelPeriod.m)

This script implements a **primal-dual method** to minimize the energy consumption of a rotary-wing UAV flying on a circular path, under multiple constraints:

- Geometric and flight limits: altitude range and coverage radius.
- Antenna beamwidth and coverage angle.
- Velocity and trajectory period limits.
- QoS constraints based on path loss and LoS probability.

This approach uses:
- Variable transformations to log-domain for convergence.
- Lagrangian updates with dual variables.
- Analytical feasibility conditions to ensure coverage.

📌 This method supports detailed scenario analysis and validates optimal configurations for long-endurance UAV missions.

---

## 🙏 Acknowledgements

This project makes extensive use of the [`sjtu_drone`](https://github.com/NovoG93/sjtu_drone) simulator developed by [NovoG93](https://github.com/NovoG93) and contributors from Shanghai Jiao Tong University.\
We sincerely thank the authors for their work and for releasing it under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html), which made this research possible.

---

## 📄 License

This repository is distributed under the **GNU General Public License v3.0 (GPL-3.0)**.\
See the [LICENSE](LICENSE) file for more information.

