# RobStride Control

This package integrates RobStride Control with ROS 2 Jazzy.

## Build & Run Instructions

1. Install ROS 2 Jazzy:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```
2. Install dependencies:
   ```bash
   pip install python-can numpy
   ```
3. Configure SocketCAN.
4. Build the package:
   ```bash
   colcon build
   ```
5. Launch the node:
   ```bash
   ros2 launch robstride_ros2 robstride.launch.py
   ```
