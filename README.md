# uare-research-trajectory-control

This repository is part of an academic research project focused on **trajectory control for UAVs (drones)** in simulated environments using **ROS 2** and **Gazebo Classic**. It builds upon the open-source [`sjtu_drone`](https://github.com/NovoG93/sjtu_drone) simulator and adds custom controllers, improved dynamics, and additional functionality for research in navigation and control.

---

## 📌 Overview

The goal of this project is to simulate, control, and evaluate various UAV trajectory tracking algorithms. Key contributions include:

- Custom control nodes using PID and experimental methods.
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
ros2 launch uare_research_trajectory_control drone_control.launch.py
```

Takeoff:

```bash
ros2 topic pub /drone/takeoff std_msgs/msg/Empty '{}' --once
```

Land:

```bash
ros2 topic pub /drone/land std_msgs/msg/Empty '{}' --once
```

