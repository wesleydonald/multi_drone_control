#!/usr/bin/env python3
"""No-authority M2A attachment observer and evidence logger.

This node publishes detector diagnostics only. It never publishes an executable
trajectory/reference. Gazebo detachable-joint truth may be logged, but it is not
passed into AttachmentObservation or AttachmentDetector.
"""

from __future__ import annotations

import csv
from datetime import datetime
import json
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, Int32, String

from tejen_mission.attachment_detection import AttachmentDetector, AttachmentObservation
from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.m2a_attachment_runtime import (
    calibration_to_dict,
    default_calibration,
    default_detector_config,
    detector_config_to_dict,
    quaternion_xyzw_to_rotation,
    NamedTransform,
    reconstruct_relative_pose_world,
    resolve_named_transform_world,
    rotation_to_quaternion_xyzw,
)


def _pose_position(pose: Pose) -> np.ndarray:
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)


def _pose_quaternion(pose: Pose) -> np.ndarray:
    return np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=float,
    )


def _select_pose(msg: PoseArray, index: int) -> Optional[Pose]:
    if not msg.poses:
        return None
    idx = int(index)
    if idx < 0:
        idx += len(msg.poses)
    if idx < 0 or idx >= len(msg.poses):
        return None
    return msg.poses[idx]


class M2AttachmentObserver(Node):
    def __init__(self) -> None:
        super().__init__("m2_attachment_observer")

        self.assigned_plate_id = int(self.declare_parameter("assigned_plate_id", 0).value)
        self.assigned_plate_topic = str(
            self.declare_parameter("assigned_plate_topic", "").value
        ).strip()
        self.pose_source = str(self.declare_parameter("pose_source", "tf_named").value).strip().lower()
        self.carrier_tf_topic = str(
            self.declare_parameter("carrier_tf_topic", "/model/x3/pose").value
        )
        self.carrier_frame_leaf = str(self.declare_parameter("carrier_frame_leaf", "base_link").value)
        self.magnet_frame_leaf = str(self.declare_parameter("magnet_frame_leaf", "magnet_tip_link").value)
        # Legacy PoseArray source remains available for replay/compatibility only.
        self.x3_pose_topic = str(self.declare_parameter("x3_pose_topic", "/model/x3/pose").value)
        self.magnet_index = int(self.declare_parameter("magnet_index", 0).value)
        self.drone_index = int(self.declare_parameter("drone_index", 7).value)
        self.x3_link_poses_are_relative = bool(
            self.declare_parameter("x3_link_poses_are_relative", True).value
        )
        self.ring_pose_source = str(self.declare_parameter("ring_pose_source", "fixed").value).strip().lower()
        self.ring_pose_topic = str(
            self.declare_parameter("ring_pose_topic", "/model/payload_model/pose").value
        )
        self.ring_pose_index = int(self.declare_parameter("ring_pose_index", 1).value)
        self.fixed_ring_x = float(self.declare_parameter("fixed_ring_x", 0.0).value)
        self.fixed_ring_y = float(self.declare_parameter("fixed_ring_y", 0.0).value)
        self.fixed_ring_z = float(self.declare_parameter("fixed_ring_z", 0.10).value)
        self.fixed_ring_yaw = float(self.declare_parameter("fixed_ring_yaw", 0.0).value)
        self.magnet_command_topic = str(
            self.declare_parameter("magnet_command_topic", "/magnet/command").value
        )
        self.proof_requested_topic = str(
            self.declare_parameter("proof_requested_topic", "/m2a/attachment/proof_requested").value
        )
        self.reset_topic = str(
            self.declare_parameter("reset_topic", "/m2a/attachment/reset").value
        )
        self.diagnostics_topic = str(
            self.declare_parameter("diagnostics_topic", "/m2a/attachment/diagnostics").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/m2a/attachment/state").value
        )
        self.confirmed_topic = str(
            self.declare_parameter("confirmed_topic", "/m2a/attachment/confirmed").value
        )
        self.probe_phase_topic = str(
            self.declare_parameter("probe_phase_topic", "/m2a/probe/phase").value
        )
        self.joint_truth_topic = str(
            self.declare_parameter("joint_truth_topic", "/m2a/sim/joint_detached_truth").value
        )
        self.publish_rate_hz = max(1.0, float(self.declare_parameter("publish_rate_hz", 50.0).value))

        defaults = default_detector_config()
        angle_direction = defaults.proof_direction_plate
        cfg = type(defaults)(
            candidate_xy_m=float(self.declare_parameter("candidate_xy_m", defaults.candidate_xy_m).value),
            candidate_normal_m=float(self.declare_parameter("candidate_normal_m", defaults.candidate_normal_m).value),
            candidate_speed_mps=float(self.declare_parameter("candidate_speed_mps", defaults.candidate_speed_mps).value),
            candidate_dwell_s=float(self.declare_parameter("candidate_dwell_s", defaults.candidate_dwell_s).value),
            proof_xy_m=float(self.declare_parameter("proof_xy_m", defaults.proof_xy_m).value),
            proof_normal_m=float(self.declare_parameter("proof_normal_m", defaults.proof_normal_m).value),
            proof_speed_mps=float(self.declare_parameter("proof_speed_mps", defaults.proof_speed_mps).value),
            proof_dwell_s=float(self.declare_parameter("proof_dwell_s", defaults.proof_dwell_s).value),
            proof_min_excitation_m=float(
                self.declare_parameter("proof_min_excitation_m", defaults.proof_min_excitation_m).value
            ),
            proof_direction_plate=np.array(
                [
                    float(self.declare_parameter("proof_direction_radial", angle_direction[0]).value),
                    float(self.declare_parameter("proof_direction_tangential", angle_direction[1]).value),
                    float(self.declare_parameter("proof_direction_normal", angle_direction[2]).value),
                ],
                dtype=float,
            ),
            loss_xy_m=float(self.declare_parameter("loss_xy_m", defaults.loss_xy_m).value),
            loss_normal_m=float(self.declare_parameter("loss_normal_m", defaults.loss_normal_m).value),
            loss_dwell_s=float(self.declare_parameter("loss_dwell_s", defaults.loss_dwell_s).value),
            pose_timeout_s=float(self.declare_parameter("pose_timeout_s", defaults.pose_timeout_s).value),
            velocity_filter_tau_s=float(
                self.declare_parameter("velocity_filter_tau_s", defaults.velocity_filter_tau_s).value
            ),
            magnet_off_counts_as_loss=bool(
                self.declare_parameter(
                    "magnet_off_counts_as_loss", defaults.magnet_off_counts_as_loss
                ).value
            ),
        )

        calibration = default_calibration()
        default_contact_q = rotation_to_quaternion_xyzw(
            calibration.rotation_magnet_from_contact
        )
        contact_q = np.array(
            [
                float(self.declare_parameter("contact_rotation_qx", default_contact_q[0]).value),
                float(self.declare_parameter("contact_rotation_qy", default_contact_q[1]).value),
                float(self.declare_parameter("contact_rotation_qz", default_contact_q[2]).value),
                float(self.declare_parameter("contact_rotation_qw", default_contact_q[3]).value),
            ], dtype=float,
        )
        calibration = type(calibration)(
            contact_position_magnet=np.array(
                [
                    float(self.declare_parameter("contact_offset_x", calibration.contact_position_magnet[0]).value),
                    float(self.declare_parameter("contact_offset_y", calibration.contact_position_magnet[1]).value),
                    float(self.declare_parameter("contact_offset_z", calibration.contact_position_magnet[2]).value),
                ],
                dtype=float,
            ),
            rotation_magnet_from_contact=quaternion_xyzw_to_rotation(contact_q),
        )

        self.contact_observation_model = str(
            self.declare_parameter("contact_observation_model", "rigid_calibrated").value
        ).strip().lower()
        self.sphere_radius_m = float(self.declare_parameter("sphere_radius_m", 0.025).value)
        self.detector = AttachmentDetector(
            geometry=RingNetGeometry(),
            assigned_plate_id=self.assigned_plate_id,
            calibration=calibration,
            config=cfg,
            contact_observation_model=self.contact_observation_model,
            sphere_radius_m=self.sphere_radius_m,
        )
        self.contact_observation_model = self.detector.contact_observation_model.value

        self.magnet_on = False
        self.proof_requested = False
        self.joint_detached_truth: Optional[bool] = None
        self.probe_phase = "STARTUP"
        self.reset_asserted = False
        self.assignment_locked = False
        self.latest_magnet_world: Optional[Tuple[np.ndarray, np.ndarray, float]] = None
        self.latest_drone_world: Optional[Tuple[np.ndarray, np.ndarray, float]] = None
        self.latest_ring_world: Optional[Tuple[np.ndarray, np.ndarray, float]] = None

        if self.pose_source == "tf_named":
            self.create_subscription(TFMessage, self.carrier_tf_topic, self._carrier_tf_callback, 20)
        elif self.pose_source == "pose_array":
            self.create_subscription(PoseArray, self.x3_pose_topic, self._x3_pose_callback, 20)
        else:
            raise ValueError("pose_source must be 'tf_named' or 'pose_array'")
        if self.ring_pose_source == "pose_array":
            self.create_subscription(PoseArray, self.ring_pose_topic, self._ring_pose_callback, 20)
        elif self.ring_pose_source != "fixed":
            raise ValueError("ring_pose_source must be 'fixed' or 'pose_array'")
        if self.assigned_plate_topic:
            self.create_subscription(Int32, self.assigned_plate_topic, self._assigned_plate_callback, 10)
        self.create_subscription(String, self.magnet_command_topic, self._magnet_command_callback, 10)
        self.create_subscription(Bool, self.proof_requested_topic, self._proof_callback, 10)
        self.create_subscription(Bool, self.reset_topic, self._reset_callback, 10)
        self.create_subscription(Bool, self.joint_truth_topic, self._truth_callback, 10)
        self.create_subscription(String, self.probe_phase_topic, self._probe_phase_callback, 10)

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.confirmed_pub = self.create_publisher(Bool, self.confirmed_topic, 10)
        self.diagnostics_pub = self.create_publisher(String, self.diagnostics_topic, 10)

        requested_log_dir = str(self.declare_parameter("log_dir", "").value).strip()
        if requested_log_dir:
            self.log_dir = Path(requested_log_dir).expanduser()
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_dir = Path("logs/m2_attachment") / f"m2a_{stamp}"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.log_dir / "attachment.csv"
        self.metadata_filename = str(self.declare_parameter("metadata_filename", "metadata.json").value).strip() or "metadata.json"
        self.csv_file = self.csv_path.open("w", newline="")
        self.fieldnames = self._fieldnames()
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        metadata = {
            "assigned_plate_id": self.assigned_plate_id,
            "detector_config": detector_config_to_dict(cfg),
            "calibration": calibration_to_dict(calibration),
            "contact_observation_model": self.contact_observation_model,
            "sphere_radius_m": self.sphere_radius_m,
            "ring_pose_source": self.ring_pose_source,
            "pose_source": self.pose_source,
            "vehicle_pose_semantics": str(
                self.declare_parameter(
                    "vehicle_pose_semantics",
                    "drone_* columns are the real X3/base_link pose; M2A bench motion may be imposed by an external support fixture",
                ).value
            ),
            "joint_truth_semantics": "raw Gazebo detachable-joint detached state; true means detached",
            "detector_truth_contract": "joint_detached_truth is logged only and is not an AttachmentObservation field",
        }
        (self.log_dir / self.metadata_filename).write_text(json.dumps(metadata, indent=2, sort_keys=True))

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._update)
        self.get_logger().info(
            f"M2 attachment observer started: plate={self.assigned_plate_id}, log={self.csv_path}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fixed_ring(self, now: float):
        c = np.cos(0.5 * self.fixed_ring_yaw)
        s = np.sin(0.5 * self.fixed_ring_yaw)
        q = np.array([0.0, 0.0, s, c], dtype=float)
        return (
            np.array([self.fixed_ring_x, self.fixed_ring_y, self.fixed_ring_z], dtype=float),
            quaternion_xyzw_to_rotation(q),
            now,
        )

    @staticmethod
    def _named_transforms_from_tf(msg: TFMessage):
        result = []
        for transform in msg.transforms:
            result.append(
                NamedTransform(
                    transform.header.frame_id,
                    transform.child_frame_id,
                    [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ],
                    [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ],
                )
            )
        return result

    def _carrier_tf_callback(self, msg: TFMessage) -> None:
        now = self._now_s()
        try:
            transforms = self._named_transforms_from_tf(msg)
            p_carrier, R_carrier, _ = resolve_named_transform_world(
                transforms, self.carrier_frame_leaf
            )
            p_magnet, R_magnet, _ = resolve_named_transform_world(
                transforms, self.magnet_frame_leaf
            )
        except (KeyError, ValueError):
            return
        self.latest_drone_world = (p_carrier, R_carrier, now)
        self.latest_magnet_world = (p_magnet, R_magnet, now)

    def _x3_pose_callback(self, msg: PoseArray) -> None:
        now = self._now_s()
        magnet_pose = _select_pose(msg, self.magnet_index)
        drone_pose = _select_pose(msg, self.drone_index)
        if magnet_pose is None or drone_pose is None:
            return
        p_drone = _pose_position(drone_pose)
        q_drone = _pose_quaternion(drone_pose)
        R_drone = quaternion_xyzw_to_rotation(q_drone)
        if self.x3_link_poses_are_relative:
            p_magnet, R_magnet = reconstruct_relative_pose_world(
                parent_position_world=p_drone,
                parent_quaternion_xyzw=q_drone,
                child_position_parent=_pose_position(magnet_pose),
                child_quaternion_xyzw=_pose_quaternion(magnet_pose),
            )
        else:
            p_magnet = _pose_position(magnet_pose)
            R_magnet = quaternion_xyzw_to_rotation(_pose_quaternion(magnet_pose))
        self.latest_drone_world = (p_drone, R_drone, now)
        self.latest_magnet_world = (p_magnet, R_magnet, now)

    def _ring_pose_callback(self, msg: PoseArray) -> None:
        pose = _select_pose(msg, self.ring_pose_index)
        if pose is None:
            return
        now = self._now_s()
        self.latest_ring_world = (
            _pose_position(pose),
            quaternion_xyzw_to_rotation(_pose_quaternion(pose)),
            now,
        )

    def _assigned_plate_callback(self, msg: Int32) -> None:
        plate = int(msg.data)
        if plate < 0 or plate >= self.detector.geometry.plate_count:
            return
        if plate == self.assigned_plate_id:
            return
        if self.assignment_locked:
            self.get_logger().error(
                f"Ignoring M2D plate reassignment after software proof: {self.assigned_plate_id} -> {plate}"
            )
            return
        self.assigned_plate_id = plate
        self.detector.assigned_plate_id = plate
        self.detector.reset()
        self.get_logger().info(f"M2D assigned plate updated to {plate}")

    def _magnet_command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command in {"ON", "1", "TRUE"}:
            self.magnet_on = True
        elif command in {"OFF", "0", "FALSE"}:
            self.magnet_on = False

    def _proof_callback(self, msg: Bool) -> None:
        self.proof_requested = bool(msg.data)

    def _reset_callback(self, msg: Bool) -> None:
        asserted = bool(msg.data)
        if asserted and not self.reset_asserted:
            self.detector.reset()
            self.proof_requested = False
            self.get_logger().info("Attachment detector attempt evidence reset")
        self.reset_asserted = asserted

    def _truth_callback(self, msg: Bool) -> None:
        self.joint_detached_truth = bool(msg.data)

    def _probe_phase_callback(self, msg: String) -> None:
        self.probe_phase = msg.data.strip() or "unknown"

    @staticmethod
    def _fieldnames():
        fields = ["wall_time_iso", "ros_time_s", "assigned_plate_id"]
        for prefix in ("ring", "magnet"):
            fields += [f"{prefix}_{a}" for a in "xyz"]
            fields += [f"{prefix}_q{a}" for a in "xyzw"]
        fields += [f"drone_{a}" for a in "xyz"]
        fields += ["ring_time_s", "magnet_time_s", "drone_time_s", "magnet_on", "proof_requested", "probe_phase"]
        fields += [
            "contact_observation_model", "magnet_center_xy_error_m", "magnet_center_normal_m", "magnet_surface_gap_m",
            "detector_state", "fresh", "radial_error_m", "tangential_error_m", "normal_error_m",
            "xy_error_m", "relative_vx_mps", "relative_vy_mps", "relative_vz_mps", "relative_speed_mps",
            "orientation_error_rad", "candidate_condition", "candidate_dwell_s", "proof_condition",
            "proof_dwell_s", "proof_excitation_m", "confirmed", "lost_condition", "geometry_separated", "loss_dwell_s", "lost",
            "candidate_xy_m", "candidate_normal_m", "candidate_speed_mps", "candidate_dwell_limit_s",
            "proof_xy_m", "proof_normal_m", "proof_speed_mps", "proof_dwell_limit_s", "proof_min_excitation_m",
            "loss_xy_m", "loss_normal_m", "loss_dwell_limit_s", "joint_detached_truth",
        ]
        return fields

    def _publish_diagnostics(self, result, *, center_plate: np.ndarray) -> None:
        cfg = self.detector.config
        candidate_ready = bool(
            result.fresh
            and result.candidate_condition
            and result.candidate_dwell_s >= cfg.candidate_dwell_s
        )
        payload = {
            "assigned_plate_id": self.assigned_plate_id,
            "contact_observation_model": self.contact_observation_model,
            "magnet_center_xy_error_m": float(np.linalg.norm(center_plate[:2])),
            "magnet_center_normal_m": float(center_plate[2]),
            "magnet_surface_gap_m": float(center_plate[2] - self.sphere_radius_m),
            "state": result.state.value,
            "fresh": bool(result.fresh),
            "radial_error_m": float(result.radial_error_m),
            "tangential_error_m": float(result.tangential_error_m),
            "normal_error_m": float(result.normal_error_m),
            "xy_error_m": float(result.xy_error_m),
            "relative_speed_mps": float(result.relative_speed_mps),
            "candidate_condition": bool(result.candidate_condition),
            "candidate_dwell_s": float(result.candidate_dwell_s),
            "candidate_ready": candidate_ready,
            "proof_condition": bool(result.proof_condition),
            "proof_dwell_s": float(result.proof_dwell_s),
            "proof_excitation_m": float(result.proof_excitation_m),
            "confirmed": bool(result.confirmed),
            "lost_condition": bool(result.lost_condition),
            "geometry_separated": bool(result.geometry_separated),
            "loss_dwell_s": float(result.loss_dwell_s),
            "lost": bool(result.lost),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.diagnostics_pub.publish(msg)

    def _update(self) -> None:
        if self.latest_magnet_world is None or self.latest_drone_world is None:
            return
        now = self._now_s()
        ring = self._fixed_ring(now) if self.ring_pose_source == "fixed" else self.latest_ring_world
        if ring is None:
            return
        p_ring, R_ring, ring_time = ring
        p_magnet, R_magnet, magnet_time = self.latest_magnet_world
        p_drone, _R_drone, drone_time = self.latest_drone_world

        # IMPORTANT: joint_detached_truth is deliberately NOT included here.
        observation = AttachmentObservation(
            time_s=now,
            ring_position_world=p_ring,
            rotation_world_from_ring=R_ring,
            ring_time_s=ring_time,
            magnet_position_world=p_magnet,
            rotation_world_from_magnet=R_magnet,
            magnet_time_s=magnet_time,
            drone_position_world=p_drone,
            drone_time_s=drone_time,
            magnet_on=self.magnet_on,
            proof_requested=self.proof_requested,
        )
        result = self.detector.update(observation)
        if result.confirmed:
            self.assignment_locked = True
        p_plate = self.detector.geometry.plate_position_world(
            ring_position=p_ring,
            rotation_world_from_ring=R_ring,
            plate_index=self.assigned_plate_id,
        )
        R_plate = self.detector.geometry.plate_rotation_world(
            rotation_world_from_ring=R_ring,
            plate_index=self.assigned_plate_id,
        )
        center_plate = R_plate.T @ (p_magnet - p_plate)

        state_msg = String(); state_msg.data = result.state.value; self.state_pub.publish(state_msg)
        confirmed_msg = Bool(); confirmed_msg.data = bool(result.confirmed); self.confirmed_pub.publish(confirmed_msg)
        self._publish_diagnostics(result, center_plate=center_plate)

        ring_q = rotation_to_quaternion_xyzw(R_ring)
        magnet_q = rotation_to_quaternion_xyzw(R_magnet)
        cfg = self.detector.config
        row = {
            "wall_time_iso": datetime.now().isoformat(timespec="milliseconds"),
            "ros_time_s": now,
            "assigned_plate_id": self.assigned_plate_id,
            "ring_time_s": ring_time,
            "magnet_time_s": magnet_time,
            "drone_time_s": drone_time,
            "magnet_on": str(self.magnet_on).lower(),
            "proof_requested": str(self.proof_requested).lower(),
            "probe_phase": self.probe_phase,
            "contact_observation_model": self.contact_observation_model,
            "magnet_center_xy_error_m": float(np.linalg.norm(center_plate[:2])),
            "magnet_center_normal_m": float(center_plate[2]),
            "magnet_surface_gap_m": float(center_plate[2] - self.sphere_radius_m),
            "detector_state": result.state.value,
            "fresh": str(result.fresh).lower(),
            "radial_error_m": result.radial_error_m,
            "tangential_error_m": result.tangential_error_m,
            "normal_error_m": result.normal_error_m,
            "xy_error_m": result.xy_error_m,
            "relative_vx_mps": result.plate_relative_velocity_mps[0],
            "relative_vy_mps": result.plate_relative_velocity_mps[1],
            "relative_vz_mps": result.plate_relative_velocity_mps[2],
            "relative_speed_mps": result.relative_speed_mps,
            "orientation_error_rad": result.orientation_error_rad,
            "candidate_condition": str(result.candidate_condition).lower(),
            "candidate_dwell_s": result.candidate_dwell_s,
            "proof_condition": str(result.proof_condition).lower(),
            "proof_dwell_s": result.proof_dwell_s,
            "proof_excitation_m": result.proof_excitation_m,
            "confirmed": str(result.confirmed).lower(),
            "lost_condition": str(result.lost_condition).lower(),
            "geometry_separated": str(result.geometry_separated).lower(),
            "loss_dwell_s": result.loss_dwell_s,
            "lost": str(result.lost).lower(),
            "candidate_xy_m": cfg.candidate_xy_m,
            "candidate_normal_m": cfg.candidate_normal_m,
            "candidate_speed_mps": cfg.candidate_speed_mps,
            "candidate_dwell_limit_s": cfg.candidate_dwell_s,
            "proof_xy_m": cfg.proof_xy_m,
            "proof_normal_m": cfg.proof_normal_m,
            "proof_speed_mps": cfg.proof_speed_mps,
            "proof_dwell_limit_s": cfg.proof_dwell_s,
            "proof_min_excitation_m": cfg.proof_min_excitation_m,
            "loss_xy_m": cfg.loss_xy_m,
            "loss_normal_m": cfg.loss_normal_m,
            "loss_dwell_limit_s": cfg.loss_dwell_s,
            "joint_detached_truth": "" if self.joint_detached_truth is None else str(self.joint_detached_truth).lower(),
        }
        for prefix, vec in (("ring", p_ring), ("magnet", p_magnet), ("drone", p_drone)):
            for i, axis in enumerate("xyz"):
                row[f"{prefix}_{axis}"] = float(vec[i])
        for prefix, quat in (("ring", ring_q), ("magnet", magnet_q)):
            for i, axis in enumerate("xyzw"):
                row[f"{prefix}_q{axis}"] = float(quat[i])
        self.writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.flush(); self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2AttachmentObserver()
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
