# Copyright 2024 Metabot contributors
# SPDX-License-Identifier: MIT
"""ROS 2 Jazzy node that drives RobStride motors via the upstream Python SDK.

It subscribes to ``/joint_commands`` (``sensor_msgs/JointState``) and publishes
``/joint_states`` at a configurable rate.  Internally it uses the
``robstride_dynamics`` driver vendored from
https://github.com/Seeed-Projects/RobStride_Control .
"""

from __future__ import annotations

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robstride_control_py.robstride_dynamics import Motor, RobstrideBus


class RobstridePyNode(Node):
    """ROS 2 wrapper around :class:`robstride_dynamics.RobstrideBus`."""

    def __init__(self) -> None:
        super().__init__('robstride_py_node')

        # --- Parameters ---
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('joint_names', ['joint_1'])
        self.declare_parameter('motor_ids', [11])
        self.declare_parameter('motor_model', 'rs-03')
        self.declare_parameter('default_kp', 50.0)
        self.declare_parameter('default_kd', 1.0)
        self.declare_parameter('publish_rate_hz', 50.0)

        self._can_interface = self.get_parameter('can_interface').value
        self._joint_names: List[str] = list(self.get_parameter('joint_names').value)
        motor_ids = list(self.get_parameter('motor_ids').value)
        self._motor_model: str = self.get_parameter('motor_model').value
        self._default_kp: float = float(self.get_parameter('default_kp').value)
        self._default_kd: float = float(self.get_parameter('default_kd').value)
        self._publish_rate_hz: float = float(self.get_parameter('publish_rate_hz').value)

        if len(self._joint_names) != len(motor_ids):
            raise RuntimeError(
                f'joint_names ({len(self._joint_names)}) and motor_ids '
                f'({len(motor_ids)}) must have the same length',
            )

        self._targets: Dict[str, float] = {name: 0.0 for name in self._joint_names}

        motors = {
            name: Motor(id=int(mid), model=self._motor_model)
            for name, mid in zip(self._joint_names, motor_ids)
        }
        self._bus = RobstrideBus(channel=self._can_interface, motors=motors)

        try:
            self._bus.connect()
            for name in self._joint_names:
                self._bus.enable(name)
            self.get_logger().info(
                f"Connected to '{self._can_interface}' with motors: {list(motors)}",
            )
            self._connected = True
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error(f'Failed to connect to CAN bus: {exc}')
            self._connected = False

        # --- ROS interfaces ---
        self._state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._cmd_sub = self.create_subscription(
            JointState, 'joint_commands', self._on_command, 10,
        )
        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

    # ------------------------------------------------------------------
    def _on_command(self, msg: JointState) -> None:
        if msg.name and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                if name in self._targets:
                    self._targets[name] = float(pos)
        elif len(msg.position) == len(self._joint_names):
            # Allow positional commands matching our joint order.
            for name, pos in zip(self._joint_names, msg.position):
                self._targets[name] = float(pos)

    def _on_timer(self) -> None:
        if not self._connected:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        positions: List[float] = []
        velocities: List[float] = []
        efforts: List[float] = []

        for name in self._joint_names:
            try:
                self._bus.write_operation_frame(
                    name,
                    position=self._targets[name],
                    kp=self._default_kp,
                    kd=self._default_kd,
                )
                pos, vel, tor, _temp = self._bus.read_operation_frame(name)
            except Exception as exc:  # pragma: no cover - hardware dependent
                self.get_logger().warning(
                    f'Communication error on joint {name!r}: {exc}',
                    throttle_duration_sec=1.0,
                )
                pos = vel = tor = math.nan

            positions.append(pos)
            velocities.append(vel)
            efforts.append(tor)

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        if self._connected:
            try:
                self._bus.disconnect(disable_torque=True)
            except Exception:  # pragma: no cover
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobstridePyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
