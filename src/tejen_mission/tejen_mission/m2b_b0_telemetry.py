#!/usr/bin/env python3
"""Structured telemetry logger for M2B B0 commissioning.

Human-readable process output remains in runtime.log. This node writes machine-
readable state to b0_status.csv so gates and later analysis do not parse ros2
`topic echo` text files.
"""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path
import time
from typing import Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from interfaces.msg import MotionCaptureState


class M2BB0Telemetry(Node):
    def __init__(self) -> None:
        super().__init__("m2b_b0_telemetry")
        self.log_dir = Path(str(self.declare_parameter("log_dir", "logs/m2_attachment").value)).expanduser()
        self.sample_rate_hz = max(1.0, float(self.declare_parameter("sample_rate_hz", 20.0).value))
        self.assigned_plate_id = int(self.declare_parameter("assigned_plate_id", 0).value)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.log_dir / "b0_status.csv"
        self.csv_file = self.csv_path.open("w", newline="")
        self.fieldnames = [
            "wall_time_s", "ros_time_s",
            "joint_detached", "arm_permission", "mission_phase",
            "requested_mpc_mode", "effective_mpc_mode", "armed",
            "drone_x", "drone_y", "drone_z", "drone_vx", "drone_vy", "drone_vz", "drone_yaw_rad",
            "magnet_x", "magnet_y", "magnet_z",
            "pendulum_phi_rad", "pendulum_theta_rad", "pendulum_length_m",
            "external_reference_fault_latched", "pendulum_fault_latched",
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()

        self.state: dict[str, Any] = {name: "" for name in self.fieldnames}
        self.start_wall = time.time()
        self.last_flush = self.start_wall

        self.create_subscription(Bool, "/m2a/sim/joint_detached_truth", self._joint, 10)
        self.create_subscription(Bool, "/join_planner/arm_permission", self._permission, 10)
        self.create_subscription(String, "/join_planner/phase", self._phase, 10)
        self.create_subscription(String, "/join_planner/mpc_mode", self._requested_mode, 10)
        self.create_subscription(String, "/tejen_mpc/mpc_mode_status", self._effective_mode, 10)
        self.create_subscription(Bool, "/drone_arming_state_feedback", self._armed, 10)
        self.create_subscription(MotionCaptureState, "/motion_capture_state", self._motion, 10)
        self.create_subscription(PoseStamped, "/magnet_tip_pose", self._magnet, 10)
        self.create_subscription(MotionCaptureState, "/pendulum_swing_state", self._pendulum, 10)
        self.create_subscription(String, "/tejen_mpc/c1d_status", self._controller_status, 10)
        self.timer = self.create_timer(1.0 / self.sample_rate_hz, self._write_row)

        metadata = {
            "schema_version": 1,
            "stage": "M2B_B0",
            "assigned_plate_id": self.assigned_plate_id,
            "csv": "b0_status.csv",
            "sample_rate_hz": self.sample_rate_hz,
            "created_wall_time_s": self.start_wall,
            "topics": {
                "joint_detached": "/m2a/sim/joint_detached_truth",
                "arm_permission": "/join_planner/arm_permission",
                "mission_phase": "/join_planner/phase",
                "requested_mpc_mode": "/join_planner/mpc_mode",
                "effective_mpc_mode": "/tejen_mpc/mpc_mode_status",
                "armed": "/drone_arming_state_feedback",
                "drone": "/motion_capture_state",
                "magnet": "/magnet_tip_pose",
                "pendulum": "/pendulum_swing_state",
                "controller_status": "/tejen_mpc/c1d_status",
            },
        }
        (self.log_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
        self.get_logger().info(f"M2B B0 structured telemetry: {self.csv_path}")

    @staticmethod
    def _yaw_from_pose(msg: MotionCaptureState) -> float:
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _joint(self, msg: Bool) -> None:
        self.state["joint_detached"] = bool(msg.data)

    def _permission(self, msg: Bool) -> None:
        self.state["arm_permission"] = bool(msg.data)

    def _phase(self, msg: String) -> None:
        self.state["mission_phase"] = str(msg.data)

    def _requested_mode(self, msg: String) -> None:
        self.state["requested_mpc_mode"] = str(msg.data)

    def _effective_mode(self, msg: String) -> None:
        self.state["effective_mpc_mode"] = str(msg.data)

    def _armed(self, msg: Bool) -> None:
        self.state["armed"] = bool(msg.data)

    def _motion(self, msg: MotionCaptureState) -> None:
        self.state.update({
            "drone_x": float(msg.pose.position.x),
            "drone_y": float(msg.pose.position.y),
            "drone_z": float(msg.pose.position.z),
            "drone_vx": float(msg.twist.linear.x),
            "drone_vy": float(msg.twist.linear.y),
            "drone_vz": float(msg.twist.linear.z),
            "drone_yaw_rad": self._yaw_from_pose(msg),
        })

    def _magnet(self, msg: PoseStamped) -> None:
        self.state.update({
            "magnet_x": float(msg.pose.position.x),
            "magnet_y": float(msg.pose.position.y),
            "magnet_z": float(msg.pose.position.z),
        })

    def _pendulum(self, msg: MotionCaptureState) -> None:
        self.state.update({
            "pendulum_phi_rad": float(msg.pose.position.x),
            "pendulum_theta_rad": float(msg.pose.position.y),
            "pendulum_length_m": float(msg.pose.position.z),
        })

    def _controller_status(self, msg: String) -> None:
        text = str(msg.data)
        external = re.search(r"(?m)^fault_latched:\s*(True|False)\b", text)
        pendulum = re.search(
            r"(?m)^pendulum_state_age_ms:.*?fault_latched:\s*(True|False)\b", text
        )
        if external is not None:
            self.state["external_reference_fault_latched"] = external.group(1) == "True"
        if pendulum is not None:
            self.state["pendulum_fault_latched"] = pendulum.group(1) == "True"

    def _write_row(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        row = dict(self.state)
        row["wall_time_s"] = time.time()
        row["ros_time_s"] = now
        self.writer.writerow(row)
        if time.time() - self.last_flush >= 0.25:
            self.csv_file.flush()
            self.last_flush = time.time()

    def destroy_node(self):
        if getattr(self, "csv_file", None) is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = M2BB0Telemetry()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
