#!/usr/bin/env python3
"""Structured M2B B1 commissioning telemetry.

Human-readable launch output belongs in runtime.log.  This node keeps B1 mission,
physical-latch and software-detector evidence in one analysis-friendly CSV.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import re
import time
from typing import Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interfaces.msg import MotionCaptureState
from std_msgs.msg import Bool, Int32, String


class M2BB1Telemetry(Node):
    def __init__(self) -> None:
        super().__init__("m2b_b1_telemetry")
        self.log_dir = Path(str(self.declare_parameter("log_dir", "logs/m2_attachment").value)).expanduser()
        self.sample_rate_hz = max(1.0, float(self.declare_parameter("sample_rate_hz", 20.0).value))
        self.assigned_plate_id = int(self.declare_parameter("assigned_plate_id", 0).value)
        self.contact_observation_model = str(
            self.declare_parameter("contact_observation_model", "sphere_center").value
        ).strip().lower()
        self.sphere_radius_m = float(self.declare_parameter("sphere_radius_m", 0.025).value)
        self.magnet_capture_gap_m = float(self.declare_parameter("magnet_capture_gap_m", 0.010).value)
        self.latch_settle_dwell_s = float(self.declare_parameter("latch_settle_dwell_s", 0.50).value)
        self.latch_settle_timeout_s = float(self.declare_parameter("latch_settle_timeout_s", 1.50).value)
        self.latch_settle_speed_mps = float(self.declare_parameter("latch_settle_speed_mps", 0.08).value)
        self.latch_settle_accel_mps2 = float(self.declare_parameter("latch_settle_accel_mps2", 2.0).value)
        self.latch_abort_speed_mps = float(self.declare_parameter("latch_abort_speed_mps", 0.50).value)
        self.latch_abort_accel_mps2 = float(self.declare_parameter("latch_abort_accel_mps2", 8.0).value)
        self.latch_abort_displacement_m = float(self.declare_parameter("latch_abort_displacement_m", 0.05).value)
        if self.contact_observation_model not in {"rigid_calibrated", "sphere_center"}:
            raise ValueError("contact_observation_model must be 'rigid_calibrated' or 'sphere_center'")
        if not math.isfinite(self.sphere_radius_m) or self.sphere_radius_m <= 0.0:
            raise ValueError("sphere_radius_m must be finite and positive")
        if not math.isfinite(self.magnet_capture_gap_m) or self.magnet_capture_gap_m <= 0.0:
            raise ValueError("magnet_capture_gap_m must be finite and positive")
        requested_proof_direction = [
            float(self.declare_parameter("proof_direction_radial", 1.0).value),
            float(self.declare_parameter("proof_direction_tangential", 0.0).value),
            float(self.declare_parameter("proof_direction_normal", 0.0).value),
        ]
        proof_norm = math.sqrt(sum(value * value for value in requested_proof_direction))
        if not math.isfinite(proof_norm) or proof_norm <= 1e-12:
            raise ValueError("proof direction must be finite and non-zero")
        self.proof_direction_plate = [value / proof_norm for value in requested_proof_direction]
        self.proof_command_m = float(self.declare_parameter("proof_command_m", 0.025).value)
        self.proof_command_speed_mps = float(self.declare_parameter("proof_command_speed_mps", 0.03).value)
        self.proof_detector_speed_limit_mps = float(self.declare_parameter("proof_detector_speed_limit_mps", 0.15).value)
        self.proof_min_excitation_m = float(self.declare_parameter("proof_min_excitation_m", 0.012).value)
        self.attached_hold_angle_deg = float(self.declare_parameter("attached_hold_angle_deg", 15.0).value)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.log_dir / "b1_status.csv"
        self.csv_file = self.csv_path.open("w", newline="")
        self.fieldnames = [
            "wall_time_s", "ros_time_s", "assigned_plate_id",
            "mission_phase", "attempt_number", "magnet_command", "proof_requested",
            "joint_detached", "capture_state", "attach_commanded",
            "latch_stable", "latch_stability_status", "latch_speed_mps",
            "latch_acceleration_mps2", "latch_displacement_m", "latch_stable_dwell_s",
            "latch_stability_reason",
            "arm_permission", "requested_mpc_mode", "effective_mpc_mode", "armed",
            "detector_state", "detector_fresh", "detector_candidate_ready",
            "detector_contact_observation_model",
            "magnet_center_xy_error_m", "magnet_center_normal_m", "magnet_surface_gap_m",
            "detector_xy_error_m", "detector_normal_error_m", "detector_relative_speed_mps",
            "proof_excitation_m", "detector_confirmed", "detector_lost",
            "drone_x", "drone_y", "drone_z", "drone_vx", "drone_vy", "drone_vz", "drone_yaw_rad",
            "magnet_x", "magnet_y", "magnet_z",
            "pendulum_phi", "pendulum_theta", "pendulum_length_m",
            "external_reference_fault_latched", "pendulum_fault_latched",
            "thrust_ratio_feedback_enabled", "thrust_ratio_feedback_applied",
            "thrust_ratio_feedback_status", "thrust_ratio_estimate",
            "thrust_ratio_estimate_std", "thrust_ratio_feedback_target",
            "mpc_thrust_ratio", "thrust_ratio_ukf_update_count",
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()
        self.state: dict[str, Any] = {field: "" for field in self.fieldnames}
        self.state["assigned_plate_id"] = self.assigned_plate_id
        self.last_flush = time.time()

        self.create_subscription(String, "/join_planner/phase", self._phase, 10)
        self.create_subscription(Int32, "/m2b/attempt_number", self._attempt, 10)
        self.create_subscription(String, "/magnet/command", self._magnet_command, 10)
        self.create_subscription(Bool, "/m2a/attachment/proof_requested", self._proof_requested, 10)
        self.create_subscription(Bool, "/m2a/sim/joint_detached_truth", self._joint, 10)
        self.create_subscription(String, "/m2a/sim/capture_state", self._capture_state, 10)
        self.create_subscription(Bool, "/m2a/sim/attach_commanded", self._attach_commanded, 10)
        self.create_subscription(Bool, "/m2b/latch_stable", self._latch_stable, 10)
        self.create_subscription(String, "/m2b/latch_stability", self._latch_stability, 10)
        self.create_subscription(Bool, "/join_planner/arm_permission", self._permission, 10)
        self.create_subscription(String, "/join_planner/mpc_mode", self._requested_mode, 10)
        self.create_subscription(String, "/tejen_mpc/mpc_mode_status", self._effective_mode, 10)
        self.create_subscription(Bool, "/drone_arming_state_feedback", self._armed, 10)
        self.create_subscription(String, "/m2a/attachment/diagnostics", self._diagnostics, 10)
        self.create_subscription(MotionCaptureState, "/motion_capture_state", self._motion, 10)
        self.create_subscription(PoseStamped, "/magnet_tip_pose", self._magnet, 10)
        self.create_subscription(MotionCaptureState, "/pendulum_swing_state", self._pendulum, 10)
        self.create_subscription(String, "/tejen_mpc/c1d_status", self._controller_status, 10)
        self.timer = self.create_timer(1.0 / self.sample_rate_hz, self._write_row)

        metadata = {
            "schema_version": 2,
            "stage": "M2B_B1",
            "assigned_plate_id": self.assigned_plate_id,
            "csv": "b1_status.csv",
            "attachment_detector_csv": "attachment.csv",
            "attachment_detector_metadata": "attachment_metadata.json",
            "sample_rate_hz": self.sample_rate_hz,
            "joint_truth_contract": "physical latch/release evidence only; never software attachment confirmation",
            "attachment_authority": "/m2a/attachment/diagnostics confirmed=true from AttachmentDetector",
            "contact_observation_model": self.contact_observation_model,
            "sphere_radius_m": self.sphere_radius_m,
            "magnet_capture_gap_m": self.magnet_capture_gap_m,
            "latch_stability_gate": {
                "settle_dwell_s": self.latch_settle_dwell_s,
                "settle_timeout_s": self.latch_settle_timeout_s,
                "settle_speed_mps": self.latch_settle_speed_mps,
                "settle_accel_mps2": self.latch_settle_accel_mps2,
                "abort_speed_mps": self.latch_abort_speed_mps,
                "abort_accel_mps2": self.latch_abort_accel_mps2,
                "abort_displacement_m": self.latch_abort_displacement_m,
            },
            "proof_direction_plate": self.proof_direction_plate,
            "proof_command_m": self.proof_command_m,
            "proof_command_speed_mps": self.proof_command_speed_mps,
            "proof_detector_speed_limit_mps": self.proof_detector_speed_limit_mps,
            "proof_min_excitation_m": self.proof_min_excitation_m,
            "attached_hold_angle_deg": self.attached_hold_angle_deg,
            "thrust_ratio_feedback_contract": (
                "full_model_kt_ukf feedback enabled in B1; CSV records live estimate, "
                "uncertainty, target, actual MPC kT, status and per-update application"
            ),
        }
        (self.log_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
        self.get_logger().info(f"M2B B1 structured telemetry: {self.csv_path}")

    @staticmethod
    def _yaw(msg: MotionCaptureState) -> float:
        q = msg.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _phase(self, msg: String) -> None: self.state["mission_phase"] = str(msg.data)
    def _attempt(self, msg: Int32) -> None: self.state["attempt_number"] = int(msg.data)
    def _magnet_command(self, msg: String) -> None: self.state["magnet_command"] = str(msg.data)
    def _proof_requested(self, msg: Bool) -> None: self.state["proof_requested"] = bool(msg.data)
    def _joint(self, msg: Bool) -> None: self.state["joint_detached"] = bool(msg.data)
    def _capture_state(self, msg: String) -> None: self.state["capture_state"] = str(msg.data)
    def _attach_commanded(self, msg: Bool) -> None: self.state["attach_commanded"] = bool(msg.data)
    def _latch_stable(self, msg: Bool) -> None: self.state["latch_stable"] = bool(msg.data)

    def _latch_stability(self, msg: String) -> None:
        try:
            data = json.loads(str(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(data, dict):
            return
        mapping = {
            "status": "latch_stability_status",
            "speed_mps": "latch_speed_mps",
            "acceleration_mps2": "latch_acceleration_mps2",
            "displacement_m": "latch_displacement_m",
            "stable_dwell_s": "latch_stable_dwell_s",
            "reason": "latch_stability_reason",
        }
        for source, target in mapping.items():
            if source in data:
                self.state[target] = data[source]
        if "ready_for_proof" in data:
            self.state["latch_stable"] = bool(data["ready_for_proof"])
    def _permission(self, msg: Bool) -> None: self.state["arm_permission"] = bool(msg.data)
    def _requested_mode(self, msg: String) -> None: self.state["requested_mpc_mode"] = str(msg.data)
    def _effective_mode(self, msg: String) -> None: self.state["effective_mpc_mode"] = str(msg.data)
    def _armed(self, msg: Bool) -> None: self.state["armed"] = bool(msg.data)

    def _diagnostics(self, msg: String) -> None:
        try:
            data = json.loads(str(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(data, dict):
            return
        mapping = {
            "state": "detector_state",
            "fresh": "detector_fresh",
            "candidate_ready": "detector_candidate_ready",
            "contact_observation_model": "detector_contact_observation_model",
            "magnet_center_xy_error_m": "magnet_center_xy_error_m",
            "magnet_center_normal_m": "magnet_center_normal_m",
            "magnet_surface_gap_m": "magnet_surface_gap_m",
            "xy_error_m": "detector_xy_error_m",
            "normal_error_m": "detector_normal_error_m",
            "relative_speed_mps": "detector_relative_speed_mps",
            "proof_excitation_m": "proof_excitation_m",
            "confirmed": "detector_confirmed",
            "lost": "detector_lost",
        }
        for source, target in mapping.items():
            if source in data:
                self.state[target] = data[source]

    def _motion(self, msg: MotionCaptureState) -> None:
        self.state.update({
            "drone_x": float(msg.pose.position.x), "drone_y": float(msg.pose.position.y), "drone_z": float(msg.pose.position.z),
            "drone_vx": float(msg.twist.linear.x), "drone_vy": float(msg.twist.linear.y), "drone_vz": float(msg.twist.linear.z),
            "drone_yaw_rad": self._yaw(msg),
        })

    def _magnet(self, msg: PoseStamped) -> None:
        self.state.update({
            "magnet_x": float(msg.pose.position.x), "magnet_y": float(msg.pose.position.y), "magnet_z": float(msg.pose.position.z),
        })

    def _pendulum(self, msg: MotionCaptureState) -> None:
        self.state.update({
            "pendulum_phi": float(msg.pose.position.x),
            "pendulum_theta": float(msg.pose.position.y),
            "pendulum_length_m": float(msg.pose.position.z),
        })

    def _controller_status(self, msg: String) -> None:
        text = str(msg.data)
        external = re.search(r"(?m)^fault_latched:\s*(True|False)\b", text)
        pendulum = re.search(r"(?m)^pendulum_state_age_ms:.*?fault_latched:\s*(True|False)\b", text)
        feedback = re.search(
            r"(?m)^thrust_ratio_feedback:\s*"
            r"enabled=(True|False)\s+applied=(True|False)\s+"
            r"status=(\S+)\s+estimate=(\S+)\s+std=(\S+)\s+"
            r"target=(\S+)\s+mpc=(\S+)\s+updates=(\d+)\s*$",
            text,
        )
        if external is not None:
            self.state["external_reference_fault_latched"] = external.group(1) == "True"
        if pendulum is not None:
            self.state["pendulum_fault_latched"] = pendulum.group(1) == "True"
        if feedback is not None:
            self.state.update({
                "thrust_ratio_feedback_enabled": feedback.group(1) == "True",
                "thrust_ratio_feedback_applied": feedback.group(2) == "True",
                "thrust_ratio_feedback_status": feedback.group(3),
                "thrust_ratio_estimate": float(feedback.group(4)),
                "thrust_ratio_estimate_std": float(feedback.group(5)),
                "thrust_ratio_feedback_target": float(feedback.group(6)),
                "mpc_thrust_ratio": float(feedback.group(7)),
                "thrust_ratio_ukf_update_count": int(feedback.group(8)),
            })

    def _write_row(self) -> None:
        row = dict(self.state)
        row["wall_time_s"] = time.time()
        row["ros_time_s"] = self.get_clock().now().nanoseconds * 1e-9
        self.writer.writerow(row)
        if time.time() - self.last_flush >= 0.25:
            self.csv_file.flush()
            self.last_flush = time.time()

    def destroy_node(self):
        if getattr(self, "csv_file", None) is not None:
            self.csv_file.flush(); self.csv_file.close(); self.csv_file = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = M2BB1Telemetry()
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
