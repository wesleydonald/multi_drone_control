#!/usr/bin/env python3
"""Phase-windowed Gazebo XY disturbance for M1 pickup-bias simulation tests.

The disturbance is intentionally applied to the X3 base link only while the
mission is in the pre-pickup stationary/slow window. It starts at
SETTLE_ABOVE_PICKUP and is cleared at LIFT_OBJECT so the fixture targets the
real M1 problem: persistent XY bias while settling over and descending to the
pickup object. The loose pickup object is never directly forced.
"""

from __future__ import annotations

import math
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def build_entity_wrench_text(
    force_x_n: float,
    force_y_n: float,
    target_link: str = "x3::X3/base_link",
) -> str:
    """Return a gz.msgs.EntityWrench text payload for one link-only XY force."""
    for value in (force_x_n, force_y_n):
        if not math.isfinite(float(value)):
            raise ValueError("disturbance force must be finite")
    if not target_link.strip():
        raise ValueError("target_link must be non-empty")
    return (
        f'entity: {{name: "{target_link}", type: 3}}, '
        "wrench: {force: {"
        f"x: {float(force_x_n)}, y: {float(force_y_n)}, z: 0.0"
        "}, torque: {x: 0.0, y: 0.0, z: 0.0}}"
    )


def build_entity_text(target_link: str = "x3::X3/base_link") -> str:
    """Return a gz.msgs.Entity payload identifying the scoped X3 base link."""
    if not target_link.strip():
        raise ValueError("target_link must be non-empty")
    return f'name: "{target_link}", type: 3'


class SimulationXYDisturbanceGate(Node):
    def __init__(self) -> None:
        super().__init__("simulation_xy_disturbance_gate")
        self.force_x_n = float(self.declare_parameter("force_x_n", 0.0).value)
        self.force_y_n = float(self.declare_parameter("force_y_n", 0.0).value)
        self.activation_phase = str(
            self.declare_parameter("activation_phase", "SETTLE_ABOVE_PICKUP").value
        ).strip()
        self.deactivation_phase = str(
            self.declare_parameter("deactivation_phase", "LIFT_OBJECT").value
        ).strip()
        self.phase_topic = str(
            self.declare_parameter("phase_topic", "/join_planner/phase").value
        ).strip()
        self.world_name = str(self.declare_parameter("world_name", "quadcopter").value).strip()
        self.target_link = str(
            self.declare_parameter("target_link", "x3::X3/base_link").value
        ).strip()
        if not self.activation_phase:
            raise ValueError("activation_phase must be non-empty")
        if not self.deactivation_phase:
            raise ValueError("deactivation_phase must be non-empty")
        if self.activation_phase == self.deactivation_phase:
            raise ValueError("activation_phase and deactivation_phase must differ")
        if not self.phase_topic:
            raise ValueError("phase_topic must be non-empty")
        if not self.world_name:
            raise ValueError("world_name must be non-empty")

        self.activation_seen = False
        self.wrench_active = False
        self.deactivation_seen = False

        phase_qos = QoSProfile(depth=1)
        phase_qos.reliability = ReliabilityPolicy.RELIABLE
        phase_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, self.phase_topic, self._on_phase, phase_qos)
        self.get_logger().info(
            "XY pickup-bias disturbance waiting for phase window "
            f"{self.activation_phase} -> {self.deactivation_phase}; "
            f"target={self.target_link}; "
            f"force=({self.force_x_n:.3f}, {self.force_y_n:.3f}) N"
        )

    def _publish_gz(self, topic: str, message_type: str, payload: str) -> bool:
        command = [
            "gz",
            "topic",
            "-t",
            topic,
            "-m",
            message_type,
            "-p",
            payload,
        ]
        try:
            result = subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f"Gazebo disturbance command failed: {exc}")
            return False

        detail = (result.stdout or result.stderr or "").strip()
        if detail:
            self.get_logger().debug(detail)
        return True

    def _apply_disturbance(self) -> None:
        self.activation_seen = True
        if abs(self.force_x_n) < 1e-12 and abs(self.force_y_n) < 1e-12:
            self.get_logger().info(
                f"Reached {self.activation_phase}; XY disturbance is zero, so no wrench was published"
            )
            return

        payload = build_entity_wrench_text(
            self.force_x_n,
            self.force_y_n,
            self.target_link,
        )
        if not self._publish_gz(
            f"/world/{self.world_name}/wrench/persistent",
            "gz.msgs.EntityWrench",
            payload,
        ):
            self.activation_seen = False
            return

        self.wrench_active = True
        self.get_logger().info(
            f"Applied persistent world-frame XY disturbance to {self.target_link} "
            f"at phase {self.activation_phase}: Fx={self.force_x_n:.3f} N, "
            f"Fy={self.force_y_n:.3f} N"
        )

    def _clear_disturbance(self) -> None:
        self.deactivation_seen = True
        if not self.wrench_active:
            self.get_logger().info(
                f"Reached {self.deactivation_phase}; no active XY disturbance needed clearing"
            )
            return

        if not self._publish_gz(
            f"/world/{self.world_name}/wrench/clear",
            "gz.msgs.Entity",
            build_entity_text(self.target_link),
        ):
            self.deactivation_seen = False
            return

        self.wrench_active = False
        self.get_logger().info(
            f"Cleared persistent XY disturbance from {self.target_link} "
            f"at phase {self.deactivation_phase}"
        )

    def _on_phase(self, message: String) -> None:
        phase = message.data.strip()
        if not self.activation_seen and phase == self.activation_phase:
            self._apply_disturbance()
            return
        if not self.deactivation_seen and phase == self.deactivation_phase:
            self._clear_disturbance()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationXYDisturbanceGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
