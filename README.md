# Metabot

This repository is used for developing the humanoid robot.

It contains an integration of the
[Seeed-Projects/RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control)
motor SDK adapted for **ROS 2 Jazzy** on Ubuntu 24.04.

## Layout

```
Metabot/
├── robstride_control_upstream/   # Vendored copy of the upstream project
│                                 # (C++, Python, Rust, Arduino — kept as reference)
└── src/                          # colcon workspace
    ├── robstride_control/        # ament_cmake C++ ROS 2 driver
    └── robstride_control_py/     # ament_python ROS 2 driver
```

The two ROS 2 packages expose the same ROS interface so you can pick whichever
language fits your application:

| Topic            | Type                       | Direction | Description                       |
| ---------------- | -------------------------- | --------- | --------------------------------- |
| `/joint_states`  | `sensor_msgs/JointState`   | publish   | Measured position / velocity / effort |
| `/joint_commands`| `sensor_msgs/JointState`   | subscribe | Target joint positions (rad)      |

Configuration (CAN interface, joint names, motor IDs, gains, rate) is set
through ROS 2 parameters — see `src/robstride_control/config/robstride.yaml`
and the matching file under `src/robstride_control_py/`.

## Prerequisites

* Ubuntu 24.04 (Noble)
* ROS 2 Jazzy Jalisco installed under `/opt/ros/jazzy`
* `can-utils` and a working SocketCAN interface
* For the Python package: `python-can`, `numpy`, `tqdm`

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-sensor-msgs \
    python3-colcon-common-extensions \
    can-utils python3-can python3-numpy python3-tqdm
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/path/to/Metabot
colcon build --symlink-install \
    --packages-select robstride_control robstride_control_py
source install/setup.bash
```

## Run

Bring the CAN bus up first (1 Mbps as in the upstream project):

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

Then launch one of the drivers:

```bash
# C++ driver
ros2 launch robstride_control robstride.launch.py

# Python driver
ros2 launch robstride_control_py robstride_py.launch.py
```

Send a position command (in radians):

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
    '{name: ["joint_1"], position: [1.57]}'
```

## Notes on the adaptation

* The upstream interactive C++ example
  (`robstride_control_upstream/cpp/src/position_control.cpp`) was refactored
  into a reusable `MotorDriver` library plus a ROS 2 node — the original file
  is kept untouched for reference.
* The upstream Python package `robstride_dynamics` is vendored under
  `src/robstride_control_py/robstride_control_py/robstride_dynamics/` and used
  directly by the rclpy node.
* The Arduino sketches and the Rust crate live under
  `robstride_control_upstream/` and are not part of the ROS 2 build; they are
  preserved so you can flash MCU firmware or experiment with the Rust SDK
  alongside the ROS 2 stack.
* Upstream license (MIT) is preserved in
  `robstride_control_upstream/LICENSE`.
