# Metabot

This repository is used for developing the humanoid robot.

It contains an integration of the
[Seeed-Projects/RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control)
motor SDK adapted for **ROS 2 Jazzy** on Ubuntu 24.04.

The project ships **two equivalent ROS 2 drivers** for RobStride motors:

* a **C++** driver (`robstride_control`)
* a **Python** driver (`robstride_control_py`)

Both expose the same ROS 2 interface, so you can pick whichever language you
prefer. This README walks through both options step by step.

---

## Repository layout

```
Metabot/
├── robstride_control_upstream/   # Vendored copy of the upstream project
│                                 # (C++, Python, Rust, Arduino — kept as reference)
└── src/                          # colcon workspace
    ├── robstride_control/        # ament_cmake C++ ROS 2 driver
    └── robstride_control_py/     # ament_python ROS 2 driver
```

## ROS 2 interface (same for C++ and Python)

| Topic             | Type                       | Direction | Description                           |
| ----------------- | -------------------------- | --------- | ------------------------------------- |
| `/joint_states`   | `sensor_msgs/JointState`   | publish   | Measured position / velocity / effort |
| `/joint_commands` | `sensor_msgs/JointState`   | subscribe | Target joint positions (rad)          |

Configuration (CAN interface, joint names, motor IDs, gains, publish rate) is
set through ROS 2 parameters. The defaults live in:

* `src/robstride_control/config/robstride.yaml` (C++)
* `src/robstride_control_py/config/robstride.yaml` (Python)

---

## 1. Prerequisites (do this once)

You need:

* Ubuntu 24.04 (Noble)
* ROS 2 Jazzy Jalisco installed under `/opt/ros/jazzy`
  ([install guide](https://docs.ros.org/en/jazzy/Installation.html))
* `can-utils` and a working SocketCAN interface (e.g. a USB‑CAN adapter
  connected to your RobStride motor)
* For the Python driver: `python-can`, `numpy`, `tqdm`

Install everything in one go:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-sensor-msgs \
    python3-colcon-common-extensions \
    can-utils python3-can python3-numpy python3-tqdm
```

## 2. Get the code

```bash
git clone https://github.com/Haziq-yusri/Metabot.git
cd Metabot
```

> The rest of this guide assumes your terminal is in the `Metabot/` directory.

## 3. Bring up the CAN bus (do this every time you reboot)

The RobStride motors run at **1 Mbps** as in the upstream project:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

Quick sanity check — you should see the interface listed as `UP`:

```bash
ip -details link show can0
```

## 4. Configure your motor (optional but recommended)

Open the YAML file for the driver you plan to use and adjust the values to
match your hardware:

* **C++:** `src/robstride_control/config/robstride.yaml`
* **Python:** `src/robstride_control_py/config/robstride.yaml`

The most important fields are:

* `can_interface` — usually `"can0"`
* `joint_names` — names you want to publish on `/joint_states`
* `motor_ids` — the CAN IDs of your motors (one per joint)
* `default_kp`, `default_kd` — position / damping gains
* `velocity_limit`, `torque_limit` — safety limits
* `publish_rate_hz` — how often `/joint_states` is published

---

## 🟦 How to use with **C++** (`robstride_control`)

### Step 1 — Build the C++ package

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robstride_control
source install/setup.bash
```

### Step 2 — Make sure the CAN bus is up

If you haven’t already, run the commands from
[Section 3](#3-bring-up-the-can-bus-do-this-every-time-you-reboot).

### Step 3 — Launch the C++ driver

```bash
ros2 launch robstride_control robstride.launch.py
```

You should see the node start and begin publishing `/joint_states`.

### Step 4 — Verify it is working

In a **new terminal** (don’t forget to `source install/setup.bash` again):

```bash
# See the measured joint state stream
ros2 topic echo /joint_states

# List the active topics
ros2 topic list
```

### Step 5 — Send a position command

In another terminal, send a target position in **radians**:

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
    '{name: ["joint_1"], position: [1.57]}'
```

To stop the driver, press `Ctrl+C` in the launch terminal.

---

## 🐍 How to use with **Python** (`robstride_control_py`)

### Step 1 — Build the Python package

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robstride_control_py
source install/setup.bash
```

> Tip: `--symlink-install` lets you edit the Python source files without
> rebuilding every time.

### Step 2 — Make sure the CAN bus is up

If you haven’t already, run the commands from
[Section 3](#3-bring-up-the-can-bus-do-this-every-time-you-reboot).

### Step 3 — Launch the Python driver

```bash
ros2 launch robstride_control_py robstride_py.launch.py
```

### Step 4 — Verify it is working

In a **new terminal** (remember `source install/setup.bash`):

```bash
ros2 topic echo /joint_states
ros2 node list
```

### Step 5 — Send a position command

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
    '{name: ["joint_1"], position: [1.57]}'
```

Press `Ctrl+C` in the launch terminal to stop the driver.

---

## (Optional) Build both drivers at once

If you want both packages available in the same workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
    --packages-select robstride_control robstride_control_py
source install/setup.bash
```

You can then launch either one — but **do not run both at the same time**
against the same CAN bus, or they will fight over the motor.

---

## Troubleshooting

* **`can0` does not exist** — your USB‑CAN adapter is not plugged in or its
  driver is missing. Run `ip link` to see the available interfaces.
* **`Operation not permitted` when bringing `can0` up** — you need `sudo`.
* **No data on `/joint_states`** — check that `motor_ids` in the YAML match
  the actual CAN ID of your motor, and that the bitrate is `1000000`.
* **`Package 'robstride_control' not found`** — you forgot to
  `source install/setup.bash` in the new terminal.
* **Two drivers running at once** — only run the C++ **or** the Python node,
  not both, since they share the CAN bus and the same topic names.

---

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
