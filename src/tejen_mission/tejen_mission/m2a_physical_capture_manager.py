#!/usr/bin/env python3
"""Independent simulator-side magnetic capture gate for M2A.

This node deliberately does NOT import attachment_detection. It decides when the
Gazebo fixed joint should physically attach using its own simulator-capture bounds.
Those bounds are separate from the software detector thresholds.
"""

from __future__ import annotations

import math
import subprocess
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, Empty, Int32, String

from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.m2a_attachment_runtime import (
    NamedTransform,
    quaternion_xyzw_to_rotation,
    reconstruct_relative_pose_world,
    resolve_named_transform_world,
)


def _pos(p: Pose):
    return np.array([p.position.x, p.position.y, p.position.z], dtype=float)


def _quat(p: Pose):
    return np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], dtype=float)


def _select(msg: PoseArray, index: int) -> Optional[Pose]:
    if not msg.poses:
        return None
    idx = index if index >= 0 else len(msg.poses) + index
    return None if idx < 0 or idx >= len(msg.poses) else msg.poses[idx]


class M2APhysicalCaptureManager(Node):
    def __init__(self) -> None:
        super().__init__("m2a_physical_capture_manager")
        self.geometry = RingNetGeometry()
        self.assigned_plate_id = int(self.declare_parameter("assigned_plate_id", 0).value)
        self.pose_source = str(self.declare_parameter("pose_source", "tf_named").value).strip().lower()
        self.carrier_tf_topic = str(
            self.declare_parameter("carrier_tf_topic", "/model/x3/pose").value
        )
        self.magnet_frame_leaf = str(self.declare_parameter("magnet_frame_leaf", "magnet_tip_link").value)
        self.x3_pose_topic = str(self.declare_parameter("x3_pose_topic", "/model/x3/pose").value)
        self.magnet_index = int(self.declare_parameter("magnet_index", 0).value)
        self.drone_index = int(self.declare_parameter("drone_index", 7).value)
        self.x3_link_poses_are_relative = bool(self.declare_parameter("x3_link_poses_are_relative", True).value)
        self.ring_pose_source = str(
            self.declare_parameter("ring_pose_source", "fixed").value
        ).strip().lower()
        self.ring_pose_topic = str(
            self.declare_parameter("ring_pose_topic", "/model/payload_model/pose").value
        )
        self.ring_pose_index = int(self.declare_parameter("ring_pose_index", 1).value)
        self.fixed_ring_x = float(self.declare_parameter("fixed_ring_x", 0.0).value)
        self.fixed_ring_y = float(self.declare_parameter("fixed_ring_y", 0.0).value)
        self.fixed_ring_z = float(self.declare_parameter("fixed_ring_z", 0.10).value)
        self.fixed_ring_yaw = float(self.declare_parameter("fixed_ring_yaw", 0.0).value)
        self.assigned_plate_topic = str(
            self.declare_parameter("assigned_plate_topic", "").value
        ).strip()
        self.contact_offset = np.array([
            float(self.declare_parameter("contact_offset_x", 0.0).value),
            float(self.declare_parameter("contact_offset_y", 0.0).value),
            float(self.declare_parameter("contact_offset_z", -0.025).value),
        ], dtype=float)
        self.contact_observation_model = str(
            self.declare_parameter("contact_observation_model", "rigid_calibrated").value
        ).strip().lower()
        if self.contact_observation_model not in {"rigid_calibrated", "sphere_center"}:
            raise ValueError("contact_observation_model must be 'rigid_calibrated' or 'sphere_center'")
        self.sphere_radius_m = float(self.declare_parameter("sphere_radius_m", 0.025).value)
        if not math.isfinite(self.sphere_radius_m) or self.sphere_radius_m <= 0.0:
            raise ValueError("sphere_radius_m must be finite and positive")

        self.physical_capture_xy_m = float(self.declare_parameter("physical_capture_xy_m", 0.025).value)
        self.physical_capture_normal_m = float(self.declare_parameter("physical_capture_normal_m", 0.025).value)
        self.physical_capture_speed_mps = float(self.declare_parameter("physical_capture_speed_mps", 0.18).value)
        self.physical_capture_dwell_s = float(self.declare_parameter("physical_capture_dwell_s", 0.10).value)
        self.physical_velocity_tau_s = float(self.declare_parameter("physical_velocity_tau_s", 0.05).value)
        self.physical_pose_timeout_s = float(self.declare_parameter("physical_pose_timeout_s", 0.25).value)
        self.ring_pose_timeout_s = float(self.declare_parameter("ring_pose_timeout_s", 0.50).value)
        self.force_initial_detach = bool(self.declare_parameter("force_initial_detach", True).value)
        self.gz_attach_topic = str(self.declare_parameter("gz_attach_topic", "/payload/attach").value)
        self.gz_detach_topic = str(self.declare_parameter("gz_detach_topic", "/payload/detach").value)
        self.magnet_command_topic = str(self.declare_parameter("magnet_command_topic", "/magnet/command").value)
        self.raw_detach_request_topic = str(
            self.declare_parameter(
                "raw_detach_request_topic", "/m2b/sim/raw_detach_request"
            ).value
        )
        self.gz_command_timeout_s = float(self.declare_parameter("gz_command_timeout_s", 1.0).value)
        self.command_backend = str(
            self.declare_parameter("command_backend", "gz_cli").value
        ).strip().lower()
        if self.command_backend not in {"gz_cli", "ros_bridge"}:
            raise ValueError("command_backend must be 'gz_cli' or 'ros_bridge'")
        self.ros_attach_command_topic = str(
            self.declare_parameter("ros_attach_command_topic", "/m2b/gz/attach").value
        )
        self.ros_detach_command_topic = str(
            self.declare_parameter("ros_detach_command_topic", "/m2b/gz/detach").value
        )
        self.ros_attach_pub = None
        self.ros_detach_pub = None
        if self.command_backend == "ros_bridge":
            self.ros_attach_pub = self.create_publisher(Empty, self.ros_attach_command_topic, 10)
            self.ros_detach_pub = self.create_publisher(Empty, self.ros_detach_command_topic, 10)

        self.magnet_on = False
        self.latest_magnet: Optional[Tuple[np.ndarray, np.ndarray, float]] = None
        self.previous_contact_plate: Optional[np.ndarray] = None
        self.previous_time: Optional[float] = None
        self.filtered_velocity = np.zeros(3)
        self.condition_start: Optional[float] = None
        self.attach_commanded = False
        self.initial_detach_sent = False
        self.latest_ring: Optional[Tuple[np.ndarray, np.ndarray, float]] = None

        if self.pose_source == "tf_named":
            self.create_subscription(TFMessage, self.carrier_tf_topic, self._tf_callback, 20)
        elif self.pose_source == "pose_array":
            self.create_subscription(PoseArray, self.x3_pose_topic, self._pose_callback, 20)
        else:
            raise ValueError("pose_source must be 'tf_named' or 'pose_array'")
        if self.ring_pose_source == "pose_array":
            self.create_subscription(PoseArray, self.ring_pose_topic, self._ring_pose_callback, 20)
        elif self.ring_pose_source != "fixed":
            raise ValueError("ring_pose_source must be 'fixed' or 'pose_array'")
        if self.assigned_plate_topic:
            self.create_subscription(Int32, self.assigned_plate_topic, self._assigned_plate_callback, 10)
        self.create_subscription(String, self.magnet_command_topic, self._magnet_command, 10)
        self.create_subscription(
            Bool, self.raw_detach_request_topic, self._raw_detach_request, 10
        )
        self.attach_commanded_topic = str(
            self.declare_parameter("attach_commanded_topic", "/m2a/sim/attach_commanded").value
        )
        self.capture_state_topic = str(
            self.declare_parameter("capture_state_topic", "/m2a/sim/capture_state").value
        )
        self.attach_pub = self.create_publisher(Bool, self.attach_commanded_topic, 10)
        self.state_pub = self.create_publisher(String, self.capture_state_topic, 10)
        self.timer = self.create_timer(0.02, self._update)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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

    def _tf_callback(self, msg: TFMessage):
        now = self._now()
        try:
            p_m, R_m, _ = resolve_named_transform_world(
                self._named_transforms_from_tf(msg), self.magnet_frame_leaf
            )
        except (KeyError, ValueError):
            return
        self.latest_magnet = (p_m, R_m, now)

    def _pose_callback(self, msg: PoseArray):
        magnet = _select(msg, self.magnet_index)
        drone = _select(msg, self.drone_index)
        if magnet is None or drone is None:
            return
        p_d = _pos(drone); q_d = _quat(drone)
        if self.x3_link_poses_are_relative:
            p_m, R_m = reconstruct_relative_pose_world(
                parent_position_world=p_d,
                parent_quaternion_xyzw=q_d,
                child_position_parent=_pos(magnet),
                child_quaternion_xyzw=_quat(magnet),
            )
        else:
            p_m = _pos(magnet); R_m = quaternion_xyzw_to_rotation(_quat(magnet))
        self.latest_magnet = (p_m, R_m, self._now())

    def _ring_pose_callback(self, msg: PoseArray):
        pose = _select(msg, self.ring_pose_index)
        if pose is None:
            return
        self.latest_ring = (
            _pos(pose),
            quaternion_xyzw_to_rotation(_quat(pose)),
            self._now(),
        )

    def _assigned_plate_callback(self, msg: Int32):
        plate = int(msg.data)
        if plate < 0 or plate >= self.geometry.plate_count:
            return
        if plate == self.assigned_plate_id:
            return
        if self.attach_commanded:
            self.get_logger().error(
                f"Ignoring M2D plate reassignment after attach command: {self.assigned_plate_id} -> {plate}"
            )
            return
        self.assigned_plate_id = plate
        self.condition_start = None
        self.previous_contact_plate = None
        self.previous_time = None
        self.filtered_velocity[:] = 0.0
        self.get_logger().info(f"M2D assigned plate updated to {plate}")

    def _magnet_command(self, msg: String):
        value = msg.data.strip().upper()
        if value in {"ON", "1", "TRUE"}:
            self.magnet_on = True
        elif value in {"OFF", "0", "FALSE"}:
            self.magnet_on = False
            self.condition_start = None
            if self.attach_commanded:
                self._command_gz(self.gz_detach_topic)
                self.attach_commanded = False

    def _raw_detach_request(self, msg: Bool):
        if not bool(msg.data):
            return
        self.condition_start = None
        if self._command_gz(self.gz_detach_topic):
            self.attach_commanded = False
            self.initial_detach_sent = True
            self._publish("raw_detach_requested")

    def _ring_pose(self):
        if self.ring_pose_source == "pose_array":
            if self.latest_ring is None:
                return None
            return self.latest_ring[0].copy(), self.latest_ring[1].copy()
        c = math.cos(self.fixed_ring_yaw); s = math.sin(self.fixed_ring_yaw)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        p = np.array([self.fixed_ring_x, self.fixed_ring_y, self.fixed_ring_z])
        return p, R

    def _command_gz(self, topic: str) -> bool:
        if self.command_backend == "ros_bridge":
            if topic == self.gz_attach_topic and self.ros_attach_pub is not None:
                self.ros_attach_pub.publish(Empty())
                return True
            if topic == self.gz_detach_topic and self.ros_detach_pub is not None:
                self.ros_detach_pub.publish(Empty())
                return True
            self.get_logger().error(f"No ROS bridge publisher configured for Gazebo topic {topic}")
            return False

        try:
            result = subprocess.run(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.gz_command_timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired:
            # Simulator actuation is evidence/control plumbing, never a reason for
            # this node to disappear. Fail closed and let the mission verify raw
            # joint truth rather than treating a command attempt as success.
            self.get_logger().error(
                f"gz command timed out for {topic} after {self.gz_command_timeout_s:.2f} s"
            )
            return False
        if result.returncode != 0:
            self.get_logger().error(f"gz command failed for {topic}: {result.stderr.strip()}")
            return False
        return True

    def _publish(self, state: str):
        b = Bool(); b.data = bool(self.attach_commanded); self.attach_pub.publish(b)
        s = String(); s.data = state; self.state_pub.publish(s)

    def _update(self):
        now = self._now()
        if self.force_initial_detach and not self.initial_detach_sent:
            if self._command_gz(self.gz_detach_topic):
                self.initial_detach_sent = True
                self.attach_commanded = False
                self._publish("initial_detach")
            return
        if self.latest_magnet is None:
            self._publish("waiting_for_pose")
            return

        p_m, R_m, pose_stamp = self.latest_magnet
        if now - pose_stamp > self.physical_pose_timeout_s:
            self.condition_start = None
            self.previous_contact_plate = None
            self.previous_time = None
            self.filtered_velocity[:] = 0.0
            self._publish("stale_pose")
            return
        ring_pose = self._ring_pose()
        if ring_pose is None:
            self._publish("waiting_for_ring_pose")
            return
        if (
            self.ring_pose_source == "pose_array"
            and self.latest_ring is not None
            and now - self.latest_ring[2] > self.ring_pose_timeout_s
        ):
            self.condition_start = None
            self._publish("stale_ring_pose")
            return
        p_ring, R_ring = ring_pose
        p_plate = self.geometry.plate_position_world(
            ring_position=p_ring,
            rotation_world_from_ring=R_ring,
            plate_index=self.assigned_plate_id,
        )
        R_plate = self.geometry.plate_rotation_world(
            rotation_world_from_ring=R_ring,
            plate_index=self.assigned_plate_id,
        )
        if self.contact_observation_model == "sphere_center":
            # Sphere orientation is intentionally ignored.  The simulated magnet
            # captures according to centre alignment, surface-to-plate gap and
            # translational centre speed, matching the useful physical quantities
            # for a freely spinning ball-jointed magnet.
            p_pc = R_plate.T @ (p_m - p_plate)
            p_pc = p_pc.copy()
            p_pc[2] -= self.sphere_radius_m
        else:
            p_contact = p_m + R_m @ self.contact_offset
            p_pc = R_plate.T @ (p_contact - p_plate)

        if self.previous_contact_plate is None or self.previous_time is None:
            velocity = np.zeros(3)
        else:
            dt = now - self.previous_time
            if dt <= 1e-6:
                velocity = self.filtered_velocity.copy()
            else:
                raw = (p_pc - self.previous_contact_plate) / dt
                alpha = 1.0 if self.physical_velocity_tau_s <= 0 else 1.0 - math.exp(-dt / self.physical_velocity_tau_s)
                velocity = (1.0 - alpha) * self.filtered_velocity + alpha * raw
        self.previous_contact_plate = p_pc.copy(); self.previous_time = now; self.filtered_velocity = velocity.copy()

        xy = float(np.linalg.norm(p_pc[:2])); normal = abs(float(p_pc[2])); speed = float(np.linalg.norm(velocity))
        condition = bool(
            self.magnet_on
            and xy <= self.physical_capture_xy_m
            and normal <= self.physical_capture_normal_m
            and speed <= self.physical_capture_speed_mps
        )
        if condition:
            if self.condition_start is None:
                self.condition_start = now
        else:
            self.condition_start = None
        dwell = 0.0 if self.condition_start is None else now - self.condition_start

        if condition and dwell >= self.physical_capture_dwell_s and not self.attach_commanded:
            if self._command_gz(self.gz_attach_topic):
                self.attach_commanded = True
                self._publish("attach_commanded")
                return
        self._publish(
            f"{'eligible' if condition else 'separated'} model={self.contact_observation_model} xy={xy:.4f} normal={normal:.4f} speed={speed:.4f} dwell={dwell:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = M2APhysicalCaptureManager()
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
