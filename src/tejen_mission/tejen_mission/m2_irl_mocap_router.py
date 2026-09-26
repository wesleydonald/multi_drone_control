"""One-socket OptiTrack UDP fanout for the minimum two-drone M2 IRL stack."""

from __future__ import annotations

from collections import deque
import json
import re
import socket
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_matrix, quaternion_multiply

from interfaces.msg import MotionCaptureState

from .m2_irl_hardware import (
    DEFAULT_MOCAP_BIND_ADDRESS,
    DEFAULT_MOCAP_BIND_PORT,
    PerRigidBodyThrottle,
)
from .motion_capture_publisher_irl import ObjectData, ParseData
from .m2_irl_launch_config import parse_identity_json


class RigidBodyParser:
    """Adapter providing one independent filter state per rigid body."""

    def __init__(self) -> None:
        self._parser = ParseData()

    def parse(self, packet: str) -> ObjectData | None:
        return self._parser.parse_packet(packet)


class M2IRLMocapRouter(Node):
    def __init__(self) -> None:
        super().__init__("m2_irl_mocap_router")
        self.bind_address = str(
            self.declare_parameter("bind_address", DEFAULT_MOCAP_BIND_ADDRESS).value
        )
        self.bind_port = int(
            self.declare_parameter("bind_port", DEFAULT_MOCAP_BIND_PORT).value
        )
        self.max_rate_hz = float(self.declare_parameter("max_rate_hz", 120.0).value)
        self.tether_anchor_body = np.array([
            float(self.declare_parameter("tether_anchor_x", 0.0).value),
            float(self.declare_parameter("tether_anchor_y", 0.0).value),
            float(self.declare_parameter("tether_anchor_z", -0.04).value),
        ])
        if not np.all(np.isfinite(self.tether_anchor_body)):
            raise ValueError("tether anchor must be finite")
        self.identity = parse_identity_json(
            str(self.declare_parameter("hardware_identity_json", "").value)
        )
        self.hardware = self.identity.vehicles
        self.ring_id = self.identity.ring_id
        self.ring_marker_translation = np.array([
            float(self.declare_parameter("ring_marker_translation_x", 0.0).value),
            float(self.declare_parameter("ring_marker_translation_y", 0.0).value),
            float(self.declare_parameter("ring_marker_translation_z", 0.0).value),
        ])
        self.ring_marker_quaternion = np.array([
            float(self.declare_parameter("ring_marker_rotation_qx", 0.0).value),
            float(self.declare_parameter("ring_marker_rotation_qy", 0.0).value),
            float(self.declare_parameter("ring_marker_rotation_qz", 0.0).value),
            float(self.declare_parameter("ring_marker_rotation_qw", 1.0).value),
        ])
        calibration_norm = float(np.linalg.norm(self.ring_marker_quaternion))
        if (
            not np.all(np.isfinite(self.ring_marker_translation))
            or not np.all(np.isfinite(self.ring_marker_quaternion))
            or calibration_norm <= 1e-12
        ):
            raise ValueError("ring-marker calibration must be finite and non-zero")
        self.ring_marker_quaternion /= calibration_norm
        self.body_ids = {vehicle_id: item.body_id for vehicle_id, item in self.hardware.items()}
        self.magnet_ids = {vehicle_id: item.magnet_id for vehicle_id, item in self.hardware.items()}
        all_ids = (self.ring_id, *self.body_ids.values(), *self.magnet_ids.values())
        if len(set(all_ids)) != len(all_ids):
            raise ValueError("ring, body and magnet rigid-body IDs must be unique")

        self.throttle = PerRigidBodyThrottle(max_rate_hz=self.max_rate_hz)
        self.parsers = {body_id: RigidBodyParser() for body_id in all_ids}
        self.latest: dict[int, ObjectData] = {}
        self.last_rx_monotonic: dict[int, float] = {}
        self.last_angles: dict[str, tuple[np.ndarray, float]] = {}
        self.rate_buffers = {
            vehicle_id: {"phi": deque(maxlen=8), "theta": deque(maxlen=8)}
            for vehicle_id in self.hardware
        }

        self.motion_pubs = {
            vehicle_id: self.create_publisher(
                MotionCaptureState, f"/{vehicle_id}/motion_capture_state", 10
            )
            for vehicle_id in self.hardware
        }
        self.magnet_pubs = {
            vehicle_id: self.create_publisher(
                PoseStamped, f"/{vehicle_id}/magnet_tip_pose", 10
            )
            for vehicle_id in self.hardware
        }
        self.pendulum_pubs = {
            vehicle_id: self.create_publisher(
                MotionCaptureState, f"/{vehicle_id}/pendulum_swing_state", 10
            )
            for vehicle_id in self.hardware
        }
        self.tf_pubs = {
            vehicle_id: self.create_publisher(
                TFMessage, f"/{vehicle_id}/mocap/transforms", 10
            )
            for vehicle_id in self.hardware
        }
        self.ring_pub = self.create_publisher(PoseArray, "/m2/ring/pose", 10)
        self.status_pub = self.create_publisher(String, "/m2/irl/mocap_status", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.bind_address, self.bind_port))
        self.create_timer(0.001, self._poll_socket)
        self.create_timer(0.1, self._publish_status)
        self.get_logger().info(
            f"M2 IRL shared mocap router bound to {self.bind_address}:{self.bind_port}; "
            f"ids={sorted(all_ids)}, pre-parse cap={self.max_rate_hz:.1f}Hz/body"
        )

    @staticmethod
    def _clean_message(message: str) -> str:
        return (
            message.replace("-(", "|")
            .replace(")-", "|")
            .replace("(", "")
            .replace(")", "")
            .strip()
            .replace("||", "|")
        )

    @staticmethod
    def _packet_id(packet: str) -> int | None:
        first = packet.split("|", 1)[0].strip()
        matches = re.findall(r"(?<!\d)(\d+)(?!\d)", first)
        return int(matches[-1]) if matches else None

    def _poll_socket(self) -> None:
        while True:
            try:
                raw, _ = self.sock.recvfrom(512)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"mocap UDP receive failed: {exc}")
                return
            packet = self._clean_message(raw.decode(errors="replace"))
            body_id = self._packet_id(packet)
            if body_id not in self.parsers:
                continue
            now = time.monotonic()
            if not self.throttle.accept(body_id, now):
                continue
            parser = self.parsers[body_id]
            obj = parser.parse(packet)
            if obj is None:
                continue
            self.latest[body_id] = obj
            self.last_rx_monotonic[body_id] = now
            self._publish_sample(body_id, obj)

    def _header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return header

    @staticmethod
    def _fill_pose(pose: Pose, obj: ObjectData) -> None:
        pose.position.x, pose.position.y, pose.position.z = map(float, obj.position)
        (
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ) = map(float, obj.rotation)

    def _fill_calibrated_ring_pose(self, pose: Pose, obj: ObjectData) -> None:
        qw, qx, qy, qz = map(float, obj.rotation)
        marker_q = np.array([qx, qy, qz, qw])
        marker_rotation = quaternion_matrix(marker_q)[:3, :3]
        ring_position = (
            np.asarray(obj.position, dtype=float)
            + marker_rotation @ self.ring_marker_translation
        )
        ring_q = quaternion_multiply(marker_q, self.ring_marker_quaternion)
        pose.position.x, pose.position.y, pose.position.z = map(float, ring_position)
        pose.orientation.x = float(ring_q[0])
        pose.orientation.y = float(ring_q[1])
        pose.orientation.z = float(ring_q[2])
        pose.orientation.w = float(ring_q[3])

    def _motion(self, obj: ObjectData) -> MotionCaptureState:
        msg = MotionCaptureState()
        msg.header = self._header()
        msg.child_frame_id = str(obj.id)
        self._fill_pose(msg.pose, obj)
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(
            float, obj.velocity
        )
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(
            float, obj.angular_velocity
        )
        return msg

    def _pose_stamped(self, obj: ObjectData) -> PoseStamped:
        msg = PoseStamped()
        msg.header = self._header()
        self._fill_pose(msg.pose, obj)
        return msg

    def _transform(self, obj: ObjectData, child: str) -> TransformStamped:
        msg = TransformStamped()
        msg.header = self._header()
        msg.child_frame_id = child
        (
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ) = map(float, obj.position)
        (
            msg.transform.rotation.w,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
        ) = map(float, obj.rotation)
        return msg

    def _publish_sample(self, body_id: int, obj: ObjectData) -> None:
        if body_id == self.ring_id:
            msg = PoseArray()
            msg.header = self._header()
            pose = Pose()
            self._fill_calibrated_ring_pose(pose, obj)
            msg.poses = [pose]
            self.ring_pub.publish(msg)
            return
        for vehicle_id in self.hardware:
            if body_id == self.body_ids[vehicle_id]:
                self.motion_pubs[vehicle_id].publish(self._motion(obj))
            elif body_id == self.magnet_ids[vehicle_id]:
                self.magnet_pubs[vehicle_id].publish(self._pose_stamped(obj))
                self._publish_pendulum(vehicle_id)
            else:
                continue
            body = self.latest.get(self.body_ids[vehicle_id])
            magnet = self.latest.get(self.magnet_ids[vehicle_id])
            if body is not None and magnet is not None:
                self.tf_pubs[vehicle_id].publish(
                    TFMessage(
                        transforms=[
                            self._transform(body, f"{vehicle_id}/base_link"),
                            self._transform(magnet, f"{vehicle_id}/magnet_tip_link"),
                        ]
                    )
                )

    def _publish_pendulum(self, vehicle_id: str) -> None:
        body = self.latest.get(self.body_ids[vehicle_id])
        magnet = self.latest.get(self.magnet_ids[vehicle_id])
        if body is None or magnet is None or abs(body.timestamp - magnet.timestamp) > 0.20:
            return
        qw, qx, qy, qz = body.rotation
        rotation = quaternion_matrix([qx, qy, qz, qw])[:3, :3]
        anchor = np.asarray(body.position) + rotation @ self.tether_anchor_body
        cable = np.asarray(magnet.position) - anchor
        length = float(np.linalg.norm(cable))
        if length < 1e-3:
            return
        angles = np.array(
            [np.arctan2(cable[0], -cable[2]), np.arctan2(cable[1], -cable[2])]
        )
        previous = self.last_angles.get(vehicle_id)
        rates = np.zeros(2)
        if previous is not None and magnet.timestamp > previous[1]:
            raw_rates = (angles - previous[0]) / (magnet.timestamp - previous[1])
            buffers = self.rate_buffers[vehicle_id]
            buffers["phi"].append(float(raw_rates[0]))
            buffers["theta"].append(float(raw_rates[1]))
            rates = np.array([np.mean(buffers["phi"]), np.mean(buffers["theta"])])
        self.last_angles[vehicle_id] = (angles, magnet.timestamp)
        msg = MotionCaptureState()
        msg.header = self._header()
        msg.child_frame_id = f"{vehicle_id}/pendulum"
        msg.pose.position.x = float(angles[0])
        msg.pose.position.y = float(angles[1])
        msg.pose.position.z = length
        msg.pose.orientation.w = 1.0
        msg.twist.linear.x, msg.twist.linear.y = float(rates[0]), float(rates[1])
        msg.twist.angular.x, msg.twist.angular.y = float(rates[0]), float(rates[1])
        self.pendulum_pubs[vehicle_id].publish(msg)

    def _publish_status(self) -> None:
        now = time.monotonic()
        ages = {
            str(body_id): (
                None
                if body_id not in self.last_rx_monotonic
                else now - self.last_rx_monotonic[body_id]
            )
            for body_id in self.parsers
        }
        msg = String()
        msg.data = json.dumps(
            {"bind": f"{self.bind_address}:{self.bind_port}", "ages_s": ages},
            sort_keys=True,
        )
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = M2IRLMocapRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
