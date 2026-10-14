#!/usr/bin/env python3
"""Magnet attachment manager for the detachable payload simulation.

This node provides a ROS-side "magnet" abstraction in front of Gazebo's raw
DetachableJoint attach/detach topics. The raw Gazebo attach topic creates the
joint immediately, regardless of where the payload currently is. This manager
only commands attachment when the simulated magnet tip is close to the pickup
object and the relative speed is low enough.

Typical first test:

  ros2 run drone_communication magnet_attachment_manager --ros-args \
    -p command_backend:=gz_cli \
    -p magnet_tip_pose_index:=-1 \
    -p object_pose_index:=0

Then toggle the magnet with:

  ros2 topic pub --once /magnet/command std_msgs/msg/String "{data: ON}"
  ros2 topic pub --once /magnet/command std_msgs/msg/String "{data: OFF}"

The default fixed-joint mode uses the existing Gazebo DetachableJoint plugin.
An experimental kinematic mode is included as a debug fallback; it repeatedly
sets the pickup object's pose to the magnet tip while attached, using the Gazebo
/world/<world>/set_pose service.
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Bool, Empty, String
from interfaces.msg import ELRSCommand, MotionCaptureState

from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from drone_magnet.contact_proof import ContactProof, ProofConfig

# his rejoin phases in which a contact may be proven (tejen_mission MissionPhase values)
CONTACT_PHASES = ('MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT', 'ATTACH_READY', 'LIFT_PROOF', 'RING_ATTACHED')

ON_WORDS = ('ON', '1', 'TRUE', 'ENABLE', 'ENABLED')
OFF_WORDS = ('OFF', '0', 'FALSE', 'DISABLE', 'DISABLED')


def _quat_rot(q, v):
    """Rotate vector v by unit quaternion q = [w, x, y, z] (plain math, no tf)."""
    w, x, y, z = q
    n = (w * w + x * x + y * y + z * z) ** 0.5 or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    vx, vy, vz = v
    # v' = v + 2*w*(u x v) + 2*(u x (u x v)),  u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return [vx + 2.0 * (w * cx + ccx), vy + 2.0 * (w * cy + ccy), vz + 2.0 * (w * cz + ccz)]


def latch_word(magnet_on: bool) -> str:
    """The elrs_interface radio latch's word for a magnet state (String ON|OFF)."""
    return 'ON' if magnet_on else 'OFF'


class MagnetAttachmentManager(Node):
    def __init__(self) -> None:
        super().__init__('magnet_attachment_manager')

        # High-level magnet interface.
        self.magnet_command_topic = self.declare_parameter('magnet_command_topic', '/magnet/command').value
        self.object_attached_topic = self.declare_parameter('object_attached_topic', '/magnet/object_attached').value
        self.attachment_state_topic = self.declare_parameter('attachment_state_topic', '/magnet/attachment_state').value

        # Pose sources. Gazebo PosePublisher often publishes PoseArray without
        # names on the ROS side, so the pose index is parameterised.
        self.magnet_tip_pose_topic = self.declare_parameter('magnet_tip_pose_topic', '/magnet_tip_pose').value
        self.object_pose_topic = self.declare_parameter('object_pose_topic', '/model/payload_model/pose').value
        self.magnet_tip_pose_index = int(self.declare_parameter('magnet_tip_pose_index', -1).value)
        self.object_pose_index = int(self.declare_parameter('object_pose_index', 0).value)
        # 'pose_array' (Gazebo model pose bridge) or 'mocap_state' (interfaces/MotionCaptureState,
        # the hardware payload rigid body on /payload/motion_capture_state).
        self.object_pose_msg_type = str(
            self.declare_parameter('object_pose_msg_type', 'pose_array').value).strip().lower()
        # OFF-CENTRE weld: shift the weld-reference point off the payload centre by this
        # world-frame XY (MUST match attach_target_publisher's x/y offset). The weld triggers
        # on tip-distance to THIS point, so with the offset the tip welds at a ring point
        # (moment arm != 0) instead of the centre -- what lets the dissipative network's
        # balanced-tension mode reconfigure the fleet. 0,0 = centre weld (central lifter).
        self.object_x_offset = float(self.declare_parameter('object_x_offset', 0.0).value)
        self.object_y_offset = float(self.declare_parameter('object_y_offset', 0.0).value)
        # the weld point's height above the ring-origin plane, in the ring frame (0 = the plane, as always;
        # card 2026-10-09_approach_tip_feedback: the approach's calibrated tip setpoint)
        self.object_z_offset = float(self.declare_parameter('object_z_offset', 0.0).value)
        self.magnet_tip_pose_msg_type = str(
            self.declare_parameter('magnet_tip_pose_msg_type', 'pose_stamped').value
        ).strip().lower()

        # Fallback object pose if the payload_model Gazebo pose is not bridged
        # into ROS yet. This matches the current world spawn pose for the red ball.
        self.use_fallback_object_pose = bool(self.declare_parameter('use_fallback_object_pose', True).value)
        self.fallback_object_x = float(self.declare_parameter('fallback_object_x', 0.0).value)
        self.fallback_object_y = float(self.declare_parameter('fallback_object_y', 0.0).value)
        self.fallback_object_z = float(self.declare_parameter('fallback_object_z', 0.08).value)

        # Attachment conditions.
        self.attach_radius = float(self.declare_parameter('attach_radius', 0.12).value)
        self.attach_speed_threshold = float(self.declare_parameter('attach_speed_threshold', 0.35).value)
        # low-pass on the relative VELOCITY vector before the speed gate (0 = off): a free
        # pendulum swing of the 3 g magnet arm averages out, a real approach speed does not
        # (sim DART ignores ball-joint damping; the arm swung ~80 s before the gate passed, R0660)
        self.rel_vel_filter_tau_s = float(self.declare_parameter('rel_vel_filter_tau_s', 0.0).value)
        # 'sim' (default) differences poses over the message stamps; 'wall' (historical) used
        # time.time(): at RTF 0.25 every speed read 4x low and the 0.05 m/s at-rest gate passed
        # a drone still descending at 0.12 m/s (R0668, 23.5 deg weld).
        self.velocity_clock = str(self.declare_parameter('velocity_clock', 'sim').value)
        self._vel_t = {}
        self._rel_vel_f = None
        self._rel_vel_t = None
        self.attach_dwell_time_s = float(self.declare_parameter('attach_dwell_time_s', 0.15).value)
        self.pose_timeout_s = float(self.declare_parameter('pose_timeout_s', 1.0).value)
        self.command_timeout_s = float(self.declare_parameter('command_timeout_s', 0.0).value)  # 0 disables timeout
        self.detach_when_magnet_off = bool(self.declare_parameter('detach_when_magnet_off', True).value)

        # Backend for low-level attach/detach. Since /payload/attach is a Gazebo
        # Transport topic in your current sim, gz_cli works without a ROS bridge.
        # Valid: 'gz_cli', 'ros_topic', 'both'.
        self.attachment_mode = str(self.declare_parameter('attachment_mode', 'fixed_joint').value).lower()
        self.command_backend = str(self.declare_parameter('command_backend', 'gz_cli').value).lower()
        self.gz_attach_topic = self.declare_parameter('gz_attach_topic', '/payload/attach').value
        self.gz_detach_topic = self.declare_parameter('gz_detach_topic', '/payload/detach').value
        self.ros_attach_topic = self.declare_parameter('ros_attach_topic', '/payload/attach').value
        self.ros_detach_topic = self.declare_parameter('ros_detach_topic', '/payload/detach').value
        self.gz_command_timeout_s = float(self.declare_parameter('gz_command_timeout_s', 1.0).value)

        # IRL electromagnet output over ELRS. This publishes a small magnet-only
        # ELRSCommand on a separate topic; the patched elrs_interface subscribes
        # to it and only copies the selected aux channel into the live CRSF packet,
        # so flight channels from the controller are preserved.
        self.enable_elrs_magnet_output = bool(
            self.declare_parameter('enable_elrs_magnet_output', True).value
        )
        self.elrs_magnet_command_topic = self.declare_parameter(
            'elrs_magnet_command_topic', '/magnet/ELRSCommand'
        ).value
        self.elrs_magnet_channel = int(self.declare_parameter('elrs_magnet_channel', 10).value)
        self.elrs_magnet_on_value = float(self.declare_parameter('elrs_magnet_on_value', 1.0).value)
        self.elrs_magnet_off_value = float(self.declare_parameter('elrs_magnet_off_value', -1.0).value)
        self.elrs_magnet_repeat_rate_hz = float(
            self.declare_parameter('elrs_magnet_repeat_rate_hz', 2.0).value
        )
        self._last_elrs_magnet_state: Optional[bool] = None
        self._last_elrs_publish_time = 0.0
        # Rig: the same ON/OFF decisions as the String radio latch of the newcomer's
        # elrs_interface (/drone_<d>/magnet), its one magnet path ('' = off, the sim).
        self.magnet_latch_topic = str(self.declare_parameter('magnet_latch_topic', '').value)

        # Optional debug-only kinematic fallback. This does not use the fixed
        # joint. It is useful only if the detachable joint plugin or bridge is
        # temporarily broken.
        self.world_name = self.declare_parameter('world_name', 'quadcopter').value
        self.kinematic_object_name = self.declare_parameter('kinematic_object_name', 'payload_model').value
        self.kinematic_z_offset = float(self.declare_parameter('kinematic_z_offset', 0.0).value)
        self.kinematic_update_rate_hz = float(self.declare_parameter('kinematic_update_rate_hz', 30.0).value)
        self.kinematic_set_pose_timeout_s = float(self.declare_parameter('kinematic_set_pose_timeout_s', 0.2).value)
        self._last_kinematic_update = 0.0
        self._last_kinematic_warning = 0.0

        # 'weld' (default): the check above declares attached. 'contact_lift' (combined M1, card
        # 2026-10-10_combined_m1_contact): the same radius, speed and plate point only make a contact
        # (and the twin's joint, sim_catch); attached needs Tejen's small lift to prove the catch.
        # Its dwells and timeout run on the node clock (sim time in the twin), not time.time().
        self.confirm_mode = str(self.declare_parameter('confirm_mode', 'weld').value).strip().lower()
        if self.confirm_mode not in ('weld', 'contact_lift'):
            raise ValueError(f'confirm_mode {self.confirm_mode!r}: weld or contact_lift')
        self.contact_dwell_s = float(self.declare_parameter('contact_dwell_s', 0.15).value)
        self.proof_d_ok = float(self.declare_parameter('proof_d_ok', 0.003).value)
        self.proof_d_fail = float(self.declare_parameter('proof_d_fail', 0.008).value)
        self.proof_h_plate = float(self.declare_parameter('proof_h_plate', 0.003).value)
        self.proof_dwell_s = float(self.declare_parameter('proof_dwell_s', 0.3).value)
        self.proof_fail_dwell_s = float(self.declare_parameter('proof_fail_dwell_s', 0.12).value)
        self.proof_timeout_s = float(self.declare_parameter('proof_timeout_s', 3.0).value)
        # rig: body-to-tip at least the taut length (Wednesday's record minus 3 mm); 0 = off (twin)
        self.proof_taut_len = float(self.declare_parameter('proof_taut_len', 0.0).value)
        self.proof_ma_s = float(self.declare_parameter('proof_ma_s', 0.1).value)
        # contact_still (critic F7; off here, on from the launch since 11 Oct: common.yaml): c0 from the dwell's last
        # contact_c0_tail_s and a vertical-speed gate contact_vz_max, so a tip still arriving does not set c0
        self.contact_still = bool(self.declare_parameter('contact_still', False).value)
        self.contact_vz_max = float(self.declare_parameter('contact_vz_max', 0.02).value)
        self.contact_c0_tail_s = float(self.declare_parameter('contact_c0_tail_s', 0.05).value)
        self.body_pose_topic = str(
            self.declare_parameter('body_pose_topic', '/drone_3/motion_capture_state').value)
        self.sim_catch = bool(self.declare_parameter('sim_catch', True).value)
        self.contact_topic = str(self.declare_parameter('contact_topic', '/magnet/contact').value)
        self.proof_state_topic = str(
            self.declare_parameter('proof_state_topic', '/magnet/attach_proof').value)
        # proof_rules (plan 2026-10-14 WP3): 'contact_lift' = the rules above; 'comotion' = Tejen's for the airborne
        # ring (comotion_proof.py: plate-frame contact gate, the tip staying with the plate on a taut tether after his
        # lift, failure only on evidence); its knobs are the comotion_* parameters
        self.proof_rules = str(self.declare_parameter('proof_rules', 'contact_lift').value).strip().lower()
        if self.proof_rules not in ('contact_lift', 'comotion'):
            raise ValueError(f'proof_rules {self.proof_rules!r}: contact_lift or comotion')
        self.lift_done_topic = str(
            self.declare_parameter('lift_done_topic', '/join_planner/proof_lift_complete').value)
        self.planner_phase_topic = str(self.declare_parameter('planner_phase_topic', '/join_planner/phase').value)
        self._lift_done = False
        self._object_rot: Optional[np.ndarray] = None
        self._proof: Optional[ContactProof] = None
        if self.confirm_mode == 'contact_lift' and self.proof_rules == 'comotion':
            defaults = ComotionConfig()
            self._proof = ComotionProof(ComotionConfig(**{
                k: float(self.declare_parameter(f'comotion_{k}', float(getattr(defaults, k))).value)
                for k in defaults.__dict__}))
        elif self.confirm_mode == 'contact_lift':
            self._proof = ContactProof(ProofConfig(
                radius=self.attach_radius, speed=self.attach_speed_threshold,
                contact_dwell_s=self.contact_dwell_s, d_ok=self.proof_d_ok, d_fail=self.proof_d_fail,
                h_plate=self.proof_h_plate, proof_dwell_s=self.proof_dwell_s,
                fail_dwell_s=self.proof_fail_dwell_s, timeout_s=self.proof_timeout_s,
                taut_len=self.proof_taut_len, ma_s=self.proof_ma_s,
                **({'c0_tail_s': self.contact_c0_tail_s, 'vz_max': self.contact_vz_max}
                   if self.contact_still else {})))
        self._t_object_clk: Optional[float] = None
        self._body_position: Optional[np.ndarray] = None
        self._t_body_clk: Optional[float] = None
        # tilt cut (card 2026-10-11_quad4_tilt_cut; '' = off): the mux's state 'cut' turns the magnet OFF even after
        # a confirmed attach (a dead QUAD4 must not hang on plate 9) and refuses every ON after it; the planner's
        # hand-over block (our drone-3 tracker latched while the partner flies it) refuses the confirmation
        self._cut = False
        self._handover_blocked = False
        cut_topic = str(self.declare_parameter('cut_state_topic', '').value)
        if cut_topic:
            self.create_subscription(String, cut_topic, self._cut_state_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE))
        # the planner's ring drop (locked: every magnet OFF, a welded QUAD4's coil too): its /magnet/command OFF is
        # refused after a confirmed attach, so the latched drop flag releases the coil like the cut ('' = off)
        self._dropped = False
        # dry hold (critic 15 Oct, F6; the coil gate holds QUAD4's coil OFF): every proof still runs, and a proven
        # one is a false proof by construction: logged as an error, never declared attached
        self._dry_run = bool(self.declare_parameter('dry_run', False).value)
        dropped_topic = str(self.declare_parameter('payload_dropped_topic', '/fleet/payload_dropped').value)
        if dropped_topic:
            self.create_subscription(Bool, dropped_topic, self._dropped_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE))
        block_topic = str(self.declare_parameter('handover_block_topic', '').value)
        if block_topic:
            self.create_subscription(Bool, block_topic, self._handover_block_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE))

        # State.
        self.magnet_on = False
        self.last_magnet_command_time: Optional[float] = None
        self.attached = False
        self.attach_command_sent = False
        self.detach_command_sent = False
        self.attach_condition_start: Optional[float] = None
        self.last_distance = float('nan')
        self.last_relative_speed = float('nan')
        self.last_state_text = ''

        self.magnet_tip_position: Optional[np.ndarray] = None
        self.object_position: Optional[np.ndarray] = None
        self.prev_magnet_tip_position: Optional[np.ndarray] = None
        self.prev_object_position: Optional[np.ndarray] = None
        self.magnet_tip_velocity = np.zeros(3, dtype=float)
        self.object_velocity = np.zeros(3, dtype=float)
        self.last_magnet_tip_time: Optional[float] = None
        self.last_object_time: Optional[float] = None

        self.create_subscription(String, self.magnet_command_topic,
                                 self._contact_command_callback if self._proof is not None
                                 else self.magnet_command_callback, 10)
        if self.magnet_tip_pose_msg_type in ('pose_stamped', 'posestamped', 'stamped'):
            self.create_subscription(PoseStamped, self.magnet_tip_pose_topic, self.magnet_tip_pose_stamped_callback, 10)
        elif self.magnet_tip_pose_msg_type in ('pose_array', 'posearray', 'array'):
            self.create_subscription(PoseArray, self.magnet_tip_pose_topic, self.magnet_tip_pose_callback, 10)
        else:
            self.get_logger().warn(
                f"Unknown magnet_tip_pose_msg_type={self.magnet_tip_pose_msg_type!r}; "
                "defaulting to PoseStamped."
            )
            self.create_subscription(PoseStamped, self.magnet_tip_pose_topic, self.magnet_tip_pose_stamped_callback, 10)
        if self.object_pose_msg_type in ('mocap_state', 'motion_capture_state', 'mocap'):
            self.create_subscription(MotionCaptureState, self.object_pose_topic,
                                     self.object_state_callback, 10)
        else:
            self.create_subscription(PoseArray, self.object_pose_topic, self.object_pose_callback, 10)

        self.attach_pub = self.create_publisher(Empty, self.ros_attach_topic, 5)
        self.detach_pub = self.create_publisher(Empty, self.ros_detach_topic, 5)
        self.attached_pub = self.create_publisher(Bool, self.object_attached_topic, 5)
        self.state_pub = self.create_publisher(String, self.attachment_state_topic, 5)
        self.elrs_magnet_pub = self.create_publisher(ELRSCommand, self.elrs_magnet_command_topic, 5)
        self.magnet_latch_pub = (self.create_publisher(String, self.magnet_latch_topic, 5)
                                 if self.magnet_latch_topic else None)
        if self._proof is not None:
            self.contact_pub = self.create_publisher(
                Bool, self.contact_topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.proof_state_pub = self.create_publisher(String, self.proof_state_topic, 5)
            self.create_subscription(MotionCaptureState, self.body_pose_topic, self._body_callback, 10)
            if isinstance(self._proof, ComotionProof):
                self.create_subscription(Bool, self.lift_done_topic,
                                         lambda msg: setattr(self, '_lift_done', bool(msg.data)), 10)
                self.create_subscription(String, self.planner_phase_topic, self._planner_phase_cb,
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                    reliability=ReliabilityPolicy.RELIABLE))

        # Gazebo's DetachableJoint is created ATTACHED at spawn, which would rigidly weld the
        # magnet to the payload from t=0 (before any ATTACH) and drag the payload around. Send
        # a one-shot detach shortly after start so the magnet begins FREE; it then only welds
        # when magnet_on + proximity trigger command_attach_once. Delay lets the joint exist.
        self.detach_on_start = bool(self.declare_parameter('detach_on_start', True).value)
        self.detach_on_start_delay_s = float(
            self.declare_parameter('detach_on_start_delay_s', 1.0).value)
        # Re-send detach at this rate (not a one-shot / fixed window): the joint may spawn AFTER
        # this node starts, so we keep freeing it until the magnet is first commanded ON.
        self.detach_on_start_period_s = float(
            self.declare_parameter('detach_on_start_period_s', 1.0).value)
        self._start_time = time.time()
        self._last_start_detach = 0.0
        self._last_contact_beat = 0.0
        self._ever_magnet_on = False
        # contact_lift with a welded start (the full M1): the start weld is reported attached and released
        # by the first OFF (the detach) like the weld check's; only the contacts after it need the proof
        self._start_weld = False
        # the drone starts WELDED to the payload (the partner demo's fourth carrier): keep
        # the joint gz created at spawn and report attached from t=0; magnet OFF releases it
        if bool(self.declare_parameter('attached_at_start', False).value):
            self.detach_on_start = False
            self.attached = True
            self.magnet_on = True
            self._ever_magnet_on = True
            if self._proof is not None:
                self._start_weld = True
                self.get_logger().warn('attached_at_start: the drone starts welded (attached until the first '
                                       'OFF, which releases it); every contact after that is proved by the lift')

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        if isinstance(self._proof, ComotionProof):
            self.proof_timer = self.create_timer(0.1, self._publish_proof_state)
            c = self._proof.config
            self.get_logger().info(
                f'proof_rules=comotion: contact xy <= {c.contact_xy:.3f} m, normal {c.normal_min:+.3f}..'
                f'{c.normal_max:+.3f} m, {c.speed:.3f} m/s for {c.contact_dwell_s:.2f} s; proven {c.proof_dwell_s:.1f} s '
                f'after the lift: drift <= {c.d_ok:.3f} m, taut >= free hang - {c.taut_margin:.3f} m (fallback '
                f'{c.taut_fallback:.3f}), '
                + (f'plate moved >= {c.comotion_range:.3f} m with the tip following (drift <= {c.follow_ratio:.1f} x '
                   'the plate\'s motion); ' if c.require_motion > 0.0 else 'no plate motion needed (twin); ')
                + f'lost at {c.lost:.3f} m for '
                f'{c.lost_dwell_s:.2f} s; sim_catch={self.sim_catch}, body={self.body_pose_topic}')
        elif self._proof is not None:
            self.proof_timer = self.create_timer(0.1, self._publish_proof_state)
            self.get_logger().info(
                f'confirm_mode=contact_lift: contact {self.attach_radius:.3f} m / '
                f'{self.attach_speed_threshold:.3f} m/s for {self.contact_dwell_s:.2f} s, proof '
                f'd_ok={self.proof_d_ok:.4f} d_fail={self.proof_d_fail:.4f} h_plate={self.proof_h_plate:.4f} '
                f'dwell={self.proof_dwell_s:.2f} s fail_dwell={self.proof_fail_dwell_s:.2f} s '
                f'timeout={self.proof_timeout_s:.1f} s taut_len={self.proof_taut_len:.3f} '
                f'ma={self.proof_ma_s:.2f} s, sim_catch={self.sim_catch}, body={self.body_pose_topic}'
                + (f', contact_still: c0 from the last {self.contact_c0_tail_s:.2f} s, vertical speed <= '
                   f'{self.contact_vz_max:.3f} m/s' if self.contact_still else ''))

        self.get_logger().info(
            'Magnet attachment manager started. '
            f'mode={self.attachment_mode}, backend={self.command_backend}, '
            f'magnet_tip_topic={self.magnet_tip_pose_topic}[{self.magnet_tip_pose_index}], '
            f'magnet_tip_type={self.magnet_tip_pose_msg_type}, '
            f'object_topic={self.object_pose_topic}[{self.object_pose_index}], '
            f'attach_radius={self.attach_radius:.3f} m, '
            f'attach_speed_threshold={self.attach_speed_threshold:.3f} m/s, '
            f'elrs_output={self.enable_elrs_magnet_output}, '
            f'elrs_topic={self.elrs_magnet_command_topic}, '
            f'elrs_channel={self.elrs_magnet_channel}, '
            f'latch_topic={self.magnet_latch_topic or "off"}'
        )

    @staticmethod
    def _pose_to_position(pose: Pose) -> np.ndarray:
        return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

    @staticmethod
    def _select_pose(msg: PoseArray, index: int) -> Optional[Pose]:
        if not msg.poses:
            return None
        if index < 0:
            index = len(msg.poses) + index
        if index < 0 or index >= len(msg.poses):
            return None
        return msg.poses[index]

    def _update_position_and_velocity(
        self,
        new_position: np.ndarray,
        now: float,
        old_position: Optional[np.ndarray],
        old_time: Optional[float],
        key: Optional[str] = None,
        stamp: Optional[float] = None,
        old_velocity: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if key is not None and self.velocity_clock == 'sim' and stamp is not None:
            t_old = self._vel_t.get(key)
            if old_position is None or t_old is None:
                self._vel_t[key] = stamp
                return new_position, np.zeros(3, dtype=float)
            dt = stamp - t_old
            if dt < 1e-4:          # same sim tick (or out of order): keep the last estimate
                return new_position, (np.zeros(3) if old_velocity is None else old_velocity)
            self._vel_t[key] = stamp
            return new_position, (new_position - old_position) / dt
        if old_position is None or old_time is None:
            return new_position, np.zeros(3, dtype=float)
        dt = max(1e-6, now - old_time)
        return new_position, (new_position - old_position) / dt

    def _stamp(self, msg) -> Optional[float]:
        h = getattr(msg, 'header', None)
        if h is not None and (h.stamp.sec or h.stamp.nanosec):
            return h.stamp.sec + h.stamp.nanosec * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9

    def magnet_tip_pose_callback(self, msg: PoseArray) -> None:
        pose = self._select_pose(msg, self.magnet_tip_pose_index)
        if pose is None:
            self.get_logger().warn(
                f'No pose at magnet_tip_pose_index={self.magnet_tip_pose_index} in {self.magnet_tip_pose_topic}; '
                f'PoseArray length={len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return
        now = time.time()
        new_pos = self._pose_to_position(pose)
        self.prev_magnet_tip_position = self.magnet_tip_position
        self.magnet_tip_position, self.magnet_tip_velocity = self._update_position_and_velocity(
            new_pos, now, self.magnet_tip_position, self.last_magnet_tip_time,
            'tip', self._stamp(msg), self.magnet_tip_velocity
        )
        self.last_magnet_tip_time = now
        if self._proof is not None:
            self._proof_sample()

    def magnet_tip_pose_stamped_callback(self, msg: PoseStamped) -> None:
        """Update magnet tip from a world-frame PoseStamped, e.g. /magnet_tip_pose."""
        now = time.time()
        new_pos = self._pose_to_position(msg.pose)
        self.prev_magnet_tip_position = self.magnet_tip_position
        self.magnet_tip_position, self.magnet_tip_velocity = self._update_position_and_velocity(
            new_pos, now, self.magnet_tip_position, self.last_magnet_tip_time,
            'tip', self._stamp(msg), self.magnet_tip_velocity
        )
        self.last_magnet_tip_time = now
        if self._proof is not None:
            self._proof_sample()

    def object_pose_callback(self, msg: PoseArray) -> None:
        pose = self._select_pose(msg, self.object_pose_index)
        if pose is None:
            self.get_logger().warn(
                f'No pose at object_pose_index={self.object_pose_index} in {self.object_pose_topic}; '
                f'PoseArray length={len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return
        self._update_object_pose(pose, self._stamp(msg))

    def object_state_callback(self, msg: MotionCaptureState) -> None:
        self._update_object_pose(msg.pose, self._stamp(msg))

    def _update_object_pose(self, pose: Pose, stamp: Optional[float] = None) -> None:
        now = time.time()
        new_pos = self._pose_to_position(pose)
        # off-centre weld reference (see __init__), rotated into the payload's attitude so
        # the trigger point is the physical rim magnet even when the disc is tilted
        q = pose.orientation
        off = _quat_rot([q.w, q.x, q.y, q.z], [self.object_x_offset, self.object_y_offset, self.object_z_offset])
        self._object_rot = np.column_stack([_quat_rot([q.w, q.x, q.y, q.z], e)
                                            for e in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])])
        new_pos[0] += off[0]
        new_pos[1] += off[1]
        new_pos[2] += off[2]
        self.prev_object_position = self.object_position
        self.object_position, self.object_velocity = self._update_position_and_velocity(
            new_pos, now, self.object_position, self.last_object_time,
            'object', stamp, self.object_velocity
        )
        self.last_object_time = now
        if self._proof is not None:
            self._t_object_clk = self._clock_s()

    def _set_elrs_channel(self, msg: ELRSCommand, channel_index: int, value: float) -> bool:
        """Set one normalised ELRS channel field, e.g. channel_10 = +1.0."""
        field_name = f'channel_{channel_index}'
        if not hasattr(msg, field_name):
            self.get_logger().warn(
                f'Cannot set {field_name} on ELRSCommand; check the message definition.',
                throttle_duration_sec=2.0,
            )
            return False
        setattr(msg, field_name, float(max(-1.0, min(1.0, value))))
        return True

    def publish_elrs_magnet_command(self, magnet_on: bool, reason: str = '', force: bool = False) -> None:
        """Publish the magnet ON/OFF: the aux channel for the patched ELRS interface and/or
        the String radio latch (magnet_latch_topic)."""
        latch = getattr(self, 'magnet_latch_pub', None)
        if not self.enable_elrs_magnet_output and latch is None:
            return

        now = time.time()
        repeat_period = 1.0 / max(0.01, self.elrs_magnet_repeat_rate_hz)
        if (
            not force
            and self._last_elrs_magnet_state is magnet_on
            and now - self._last_elrs_publish_time < repeat_period
        ):
            return

        value = self.elrs_magnet_on_value if magnet_on else self.elrs_magnet_off_value
        if self.enable_elrs_magnet_output:
            msg = ELRSCommand()
            # These fields are ignored by the patched elrs_interface magnet callback,
            # but fill safe defaults so the message is still inspectable in ROS echo.
            msg.armed = False
            for idx in [0, 1, 3, 4, 5, 6, 7, 8, 9, 10]:
                self._set_elrs_channel(msg, idx, 0.0)
            self._set_elrs_channel(msg, 2, -1.0)
            if not all(self._set_elrs_channel(msg, ch, value) for ch in range(5, 11)):
                return
            self.elrs_magnet_pub.publish(msg)
        if latch is not None:
            latch.publish(String(data=latch_word(magnet_on)))

        self._last_elrs_publish_time = now
        changed = self._last_elrs_magnet_state is not magnet_on
        self._last_elrs_magnet_state = magnet_on
        if changed:      # forced re-sends (the planner's keepalive) are not logged: T2 noise, plan WP2
            state = 'ON' if magnet_on else 'OFF'
            suffix = f' ({reason})' if reason else ''
            where = (f'channel_{self.elrs_magnet_channel}={value:+.2f}'
                     if self.enable_elrs_magnet_output else '')
            if latch is not None:
                where = (where + ' ' if where else '') + f'{self.magnet_latch_topic}={state}'
            self.get_logger().info(f'ELRS magnet {state}: {where}{suffix}')

    def magnet_command_callback(self, msg: String) -> None:
        command = str(msg.data).strip().upper()
        self.last_magnet_command_time = time.time()

        if command in ON_WORDS and (getattr(self, '_cut', False) or getattr(self, '_dropped', False)):
            self.get_logger().warn('Magnet ON refused: this drone was cut or the ring was dropped',
                                   throttle_duration_sec=2.0)
            return
        if command in ('ON', '1', 'TRUE', 'ENABLE', 'ENABLED'):
            if not self.magnet_on:
                self.get_logger().info('Magnet command: ON')
            self.magnet_on = True
            self._ever_magnet_on = True  # stop the startup keep-free retry; a real attach is wanted
            self.detach_command_sent = False
            self.publish_elrs_magnet_command(True, 'magnet command ON', force=True)

        elif command in ('OFF', '0', 'FALSE', 'DISABLE', 'DISABLED'):
            if self.magnet_on:
                self.get_logger().info('Magnet command: OFF')
            self.magnet_on = False
            self.attach_condition_start = None
            self.attach_command_sent = False
            self.publish_elrs_magnet_command(False, 'magnet command OFF', force=True)
            if self.detach_when_magnet_off:
                self.command_detach_once('magnet command OFF')

        else:
            self.get_logger().warn(f'Ignoring unknown magnet command: {msg.data!r}')

    def current_object_position(self) -> Optional[np.ndarray]:
        if self.object_position is not None:
            return self.object_position.copy()
        if self.use_fallback_object_pose:
            return np.array([self.fallback_object_x, self.fallback_object_y, self.fallback_object_z], dtype=float)
        return None

    def poses_fresh(self) -> Tuple[bool, str]:
        now = time.time()
        if self.magnet_tip_position is None or self.last_magnet_tip_time is None:
            return False, 'waiting for magnet tip pose'
        if now - self.last_magnet_tip_time > self.pose_timeout_s:
            return False, f'magnet tip pose stale ({now - self.last_magnet_tip_time:.2f}s)'
        if self.object_position is not None and self.last_object_time is not None:
            if now - self.last_object_time > self.pose_timeout_s:
                if not self.use_fallback_object_pose:
                    return False, f'object pose stale ({now - self.last_object_time:.2f}s)'
        elif not self.use_fallback_object_pose:
            return False, 'waiting for object pose'
        return True, 'poses fresh'

    def command_attach_once(self, reason: str) -> None:
        if self.attach_command_sent and self.attached:
            return
        self.attach_command_sent = True
        self.detach_command_sent = False
        self.attached = True  # local latch; may be overwritten later if you bridge joint state
        self.get_logger().info(f'ATTACH: {reason}')
        self.publish_elrs_magnet_command(True, f'attach: {reason}', force=True)

        if self.command_backend in ('ros_topic', 'both'):
            self.attach_pub.publish(Empty())
        if self.command_backend in ('gz_cli', 'both'):
            self.publish_gz_empty(self.gz_attach_topic)

    def command_detach_once(self, reason: str) -> None:
        if self.detach_command_sent and not self.attached:
            return
        self.detach_command_sent = True
        self.attach_command_sent = False
        self.attached = False
        self.attach_condition_start = None
        if getattr(self, '_start_weld', False):
            self._start_weld = False
            self.get_logger().info('the start weld is released: from the next ON a contact needs the lift proof')
        self.get_logger().info(f'DETACH: {reason}')
        self.publish_elrs_magnet_command(False, f'detach: {reason}', force=True)

        if self.command_backend in ('ros_topic', 'both'):
            self.detach_pub.publish(Empty())
        if self.command_backend in ('gz_cli', 'both'):
            self.publish_gz_empty(self.gz_detach_topic)

    def publish_gz_empty(self, topic: str) -> bool:
        try:
            result = subprocess.run(
                ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self.gz_command_timeout_s,
                check=False,
            )
        except Exception as exc:  # noqa: BLE001 - this is a sim utility node
            self.get_logger().error(f'Failed to call gz topic for {topic}: {exc}')
            return False

        if result.returncode != 0:
            self.get_logger().error(
                f'gz topic command failed for {topic}: returncode={result.returncode}, stderr={result.stderr.strip()}'
            )
            return False
        return True

    def update_kinematic_attachment(self, now: float) -> None:
        if self.attachment_mode != 'kinematic' or not self.attached:
            return
        if self.magnet_tip_position is None:
            return
        period = 1.0 / max(1.0, self.kinematic_update_rate_hz)
        if now - self._last_kinematic_update < period:
            return
        self._last_kinematic_update = now

        p = self.magnet_tip_position.copy()
        p[2] += self.kinematic_z_offset
        req = (
            f'name: "{self.kinematic_object_name}" '
            f'position {{ x: {p[0]:.6f} y: {p[1]:.6f} z: {p[2]:.6f} }} '
            'orientation { w: 1.0 }'
        )
        service = f'/world/{self.world_name}/set_pose'
        try:
            result = subprocess.run(
                [
                    'gz', 'service', '-s', service,
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', str(int(1000.0 * self.kinematic_set_pose_timeout_s)),
                    '--req', req,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(0.2, self.kinematic_set_pose_timeout_s + 0.1),
                check=False,
            )
            if result.returncode != 0 and now - self._last_kinematic_warning > 2.0:
                self._last_kinematic_warning = now
                self.get_logger().warn(
                    f'Kinematic set_pose failed: returncode={result.returncode}, stderr={result.stderr.strip()}'
                )
        except Exception as exc:  # noqa: BLE001
            if now - self._last_kinematic_warning > 2.0:
                self._last_kinematic_warning = now
                self.get_logger().warn(f'Kinematic set_pose exception: {exc}')

    def publish_state(self, text: str) -> None:
        attached_msg = Bool()
        attached_msg.data = bool(self.attached)
        self.attached_pub.publish(attached_msg)

        state_msg = String()
        state_msg.data = text
        self.state_pub.publish(state_msg)
        self.last_state_text = text

    # ----- confirm_mode contact_lift -----

    def _clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _body_callback(self, msg: MotionCaptureState) -> None:
        self._body_position = self._pose_to_position(msg.pose)
        self._t_body_clk = self._clock_s()

    def _cut_state_cb(self, msg: String) -> None:
        state = str(msg.data).strip().lower()
        if self._cut and state in ('partner', 'ours'):
            # only a relaunched mux leaves 'cut' ('latched' keeps it): this drone may fly again (critic round 2, L-B)
            self._cut = False
            self.get_logger().warn(f'CUT cleared: the mux reads {state} (relaunched): magnet ON allowed again')
            return
        if self._cut or state != 'cut':
            return
        self._cut = True
        if self._proof is not None:
            self._proof.state = 'off'          # the cut overrides the confirmed attach's OFF refusal
            self._proof._forget_contact()
            self._proof._forget_samples()
        self._start_weld = False
        self.get_logger().error('CUT: the mux cut this drone: magnet OFF (a confirmed attach too), no ON from now on')
        if self._proof is not None:
            self.contact_pub.publish(Bool(data=False))
        self.magnet_command_callback(String(data='OFF'))

    def _dropped_cb(self, msg: Bool) -> None:
        if not bool(msg.data) or self._dropped:
            return
        self._dropped = True
        if self._proof is not None:
            self._proof.state = 'off'          # the drop overrides the confirmed attach's OFF refusal
            self._proof._forget_contact()
            self._proof._forget_samples()
        self._start_weld = False
        self.get_logger().error('PAYLOAD DROPPED: magnet OFF (a confirmed attach too), no ON from now on')
        if self._proof is not None:
            self.contact_pub.publish(Bool(data=False))
        self.magnet_command_callback(String(data='OFF'))

    def _handover_block_cb(self, msg: Bool) -> None:
        if bool(msg.data) and not self._handover_blocked:
            self.get_logger().error('HAND-OVER BLOCKED: our tracker for this drone latched while the partner flies '
                                    'it: no attach is confirmed from now on')
        self._handover_blocked = bool(msg.data)

    def _contact_command_callback(self, msg: String) -> None:
        """The arbiter in front of magnet_command_callback."""
        word = str(msg.data).strip().upper()
        if (self._cut or self._dropped) and word in ON_WORDS:
            self.get_logger().warn('Magnet ON refused: this drone was cut or the ring was dropped',
                                   throttle_duration_sec=2.0)
            return
        if self._proof is not None:
            if word in OFF_WORDS and self._contact_off('magnet command OFF'):
                return
            if word in ON_WORDS and not self._start_weld:     # nothing to prove while the start weld holds
                self._proof.magnet(True)
        self.magnet_command_callback(msg)

    def _contact_off(self, reason: str) -> bool:
        """An OFF for the proof; True when it is refused (the attach is confirmed: nothing detaches)."""
        ev = self._proof.magnet(False)
        if ev == 'refused':
            self.get_logger().warn(f'Magnet OFF refused ({reason}): the attach is confirmed, the magnet '
                                   'stays ON', throttle_duration_sec=2.0)
            return True
        if ev == 'reset':
            self.get_logger().info(f'CONTACT RESET ({reason}): the proof is void until a new ON')
            if self.sim_catch:
                self._sim_joint(False, reason)
            self.contact_pub.publish(Bool(data=False))
        return False

    def _sim_joint(self, attach: bool, reason: str) -> None:
        """The twin's catch: the ring joint through the weld check's backend, without declaring the
        attach (not command_attach_once: attached and object_attached wait for the proof)."""
        self.get_logger().info(f'{"CATCH" if attach else "RELEASE"} (sim joint): {reason}')
        pub, topic = ((self.attach_pub, self.gz_attach_topic) if attach
                      else (self.detach_pub, self.gz_detach_topic))
        if self.command_backend in ('ros_topic', 'both'):
            pub.publish(Empty())
        if self.command_backend in ('gz_cli', 'both'):
            self.publish_gz_empty(topic)

    def _proof_sample(self) -> None:
        """One tip pose into the proof, against the latest plate point and body (each at most
        proof_ma_s old on the node clock)."""
        now = self._clock_s()
        plate_fresh = (self.object_position is not None and self._t_object_clk is not None
                       and now - self._t_object_clk <= self.proof_ma_s)
        body = (self._body_position if self._t_body_clk is not None
                and now - self._t_body_clk <= self.proof_ma_s else None)
        if isinstance(self._proof, ComotionProof):
            self._proof_event(self._proof.step(now, self.magnet_tip_position, self.object_position, body=body,
                                               fresh=plate_fresh, rotation=self._object_rot,
                                               lift_done=self._lift_done))
            return
        self._proof_event(self._proof.step(now, self.magnet_tip_position, self.object_position,
                                           body=body, fresh=plate_fresh))

    def _proof_event(self, ev: Optional[str]) -> None:
        p = self._proof
        if ev == 'contact' and isinstance(p, ComotionProof):
            hang = 'none (no taut check)' if p.l_hang is None else f'{p.l_hang:.4f} m ({p.l_hang_source})'
            self.get_logger().info(
                f'CONTACT (try {p.tries}): xy {1000.0 * p.xy:.1f} mm, normal {1000.0 * p.normal:+.1f} mm, '
                f'rel_speed={p.rel_speed:.3f} m/s, free-hang tether {hang}, plate z0={p.z0:.4f} m')
            if self.sim_catch:
                self._sim_joint(True, f'contact, try {p.tries}')
            self.contact_pub.publish(Bool(data=True))
        elif ev == 'contact':
            c0 = 1000.0 * p.c0
            self.get_logger().info(
                f'CONTACT (try {p.tries}): d={p.distance:.4f} m, rel_speed={p.rel_speed:.3f} m/s, '
                + (f'rel_vz={p.rel_vz:.3f} m/s, ' if p.config.vz_max > 0.0 else '') +
                f'c0=({c0[0]:+.1f}, {c0[1]:+.1f}, {c0[2]:+.1f}) mm, plate z0={p.z0:.4f} m')
            if self.sim_catch:
                self._sim_joint(True, f'contact, try {p.tries}')
            self.contact_pub.publish(Bool(data=True))
        elif ev == 'failed':
            f = p.last_fail
            self.get_logger().warn(
                f'PROOF FAILED (try {p.tries}, {f["reason"]}): drift {1000.0 * f["drift_m"]:.1f} mm, '
                f'plate rise {1000.0 * f["plate_rise_m"]:.1f} mm, {f["contact_age_s"]:.2f} s after the contact'
                + (f', xy {1000.0 * f["xy_m"]:.1f} mm, normal {1000.0 * f["normal_m"]:+.1f} mm' if 'xy_m' in f else ''))
            # the joint goes before the contact false, so he never descends again while welded
            if self.sim_catch:
                self._sim_joint(False, f'proof failed ({f["reason"]})')
            self.contact_pub.publish(Bool(data=False))
        elif ev == 'proven' and (self._handover_blocked or self._cut or self._dropped or self._dry_run):
            # no hand-over can follow (our tracker latched, the drone was cut, the ring dropped, a dry hold): failed
            if self._dry_run:
                self.get_logger().error(
                    f'DRY HOLD: FALSE PROOF (try {p.tries}): proven with the coil OFF (drift {1000.0 * p.drift:.1f} mm, '
                    f'normal {1000.0 * p.normal:+.1f} mm, plate moved {1000.0 * p.comotion:.1f} mm, tether '
                    f'{p.body_to_tip:.4f} m): not attached')
            p.state = 'prove'
            self._proof_event(p._fail('handover_blocked' if self._handover_blocked else 'cut' if self._cut else
                                      'dropped' if self._dropped else 'dry_run', self._clock_s()))
        elif ev == 'proven' and isinstance(p, ComotionProof):
            self._declare_attached(
                f'proven (try {p.tries}): the tip stayed with the plate (drift {1000.0 * p.drift:.1f} mm, normal '
                f'{1000.0 * p.normal:+.1f} mm) while the plate moved {1000.0 * p.comotion:.1f} mm on a taut tether '
                f'({p.body_to_tip:.4f} m, free hang {p.l_hang if p.l_hang is not None else float("nan"):.4f} m)')
        elif ev == 'proven':
            self._declare_attached(
                f'proven (try {p.tries}): drift {1000.0 * p.drift:.1f} mm, plate rise '
                f'{1000.0 * p.rise:.1f} mm, body-to-tip {p.body_to_tip:.3f} m')

    def _planner_phase_cb(self, msg: String) -> None:
        """His planner left the contact phases while we prove (a give-up, a take-back, a LAND): the contact is void."""
        phase = msg.data.strip().upper()
        if self._proof is not None and self._proof.state == 'prove' and phase not in CONTACT_PHASES:
            self._proof_event(self._proof.abort(f'aborted ({phase})', self._clock_s()))

    def _declare_attached(self, reason: str) -> None:
        """The proven catch: attached, latched (an OFF is refused from now on). The twin's joint was
        made at the contact; nothing is sent to the backend here."""
        self.attached = True
        self.attach_command_sent = True
        self.detach_command_sent = False
        self.get_logger().info(f'ATTACH: {reason}')
        self.publish_elrs_magnet_command(True, f'attach: {reason}', force=True)
        self.attached_pub.publish(Bool(data=True))

    def _contact_lift_tick(self) -> None:
        p = self._proof
        self._proof_event(p.tick(self._clock_s()))
        self.publish_state(
            f'magnet_on={self.magnet_on} attached={self.attached} mode=contact_lift state={p.state} '
            f'distance={p.distance:.3f}m radius={self.attach_radius:.3f}m '
            f'rel_speed={p.rel_speed:.3f}m/s threshold={self.attach_speed_threshold:.3f}m/s '
            f'tries={p.tries} fails={p.fails}')

    def _publish_proof_state(self) -> None:
        if self._proof is not None:
            self.proof_state_pub.publish(String(data=json.dumps(self._proof.report(self._clock_s()))))

    def timer_callback(self) -> None:
        now = time.time()

        # Keep the magnet FREE until it is first commanded ON. gz's DetachableJoint is created
        # ATTACHED at spawn, so without this the magnet is welded to the payload from t=0 and
        # drags it around (the pendulum gets pulled sideways instead of hanging straight down).
        # We re-send detach at a low rate -- not a one-shot / fixed window -- because the joint
        # may spawn AFTER this node starts, so a single early detach can be lost. Stops the
        # instant the magnet is first commanded ON, so it never fights a real attach.
        elapsed = now - self._start_time
        if (self.detach_on_start and not self._ever_magnet_on and not self.magnet_on
                and elapsed >= self.detach_on_start_delay_s
                and now - self._last_start_detach >= self.detach_on_start_period_s):
            self._last_start_detach = now
            self.attached = False
            self.get_logger().info(
                'Startup detach: keeping the auto-created magnet joint FREE until ATTACH',
                throttle_duration_sec=3.0)
            if self.command_backend in ('ros_topic', 'both'):
                self.detach_pub.publish(Empty())
            if self.command_backend in ('gz_cli', 'both'):
                self.publish_gz_empty(self.gz_detach_topic)

        # his planner reads the contact as a level: a 1 Hz repeat bounds a lost change to 1 s
        if self._proof is not None and now - self._last_contact_beat >= 1.0:
            self._last_contact_beat = now
            self.contact_pub.publish(Bool(data=self._proof.state in ('prove', 'attached')))

        if self.command_timeout_s > 0.0 and self.last_magnet_command_time is not None:
            if now - self.last_magnet_command_time > self.command_timeout_s and not (
                    self._proof is not None and self._contact_off('magnet command timeout')):
                if self.magnet_on:
                    self.get_logger().warn('Magnet command timed out; turning magnet OFF')
                self.magnet_on = False
                self.publish_elrs_magnet_command(False, 'magnet command timeout')
                if self.detach_when_magnet_off:
                    self.command_detach_once('magnet command timeout')

        # Keep the physical ELRS magnet channel alive at a low rate even if the
        # high-level command is latched. This helps if the ELRS interface starts
        # after this node or misses the first command.
        self.publish_elrs_magnet_command(self.magnet_on)

        if self._proof is not None:
            self._contact_lift_tick()
            return

        fresh, freshness_reason = self.poses_fresh()
        object_pos = self.current_object_position()
        if not fresh or object_pos is None or self.magnet_tip_position is None:
            self.publish_state(
                f'magnet_on={self.magnet_on} attached={self.attached} status={freshness_reason}'
            )
            return

        rel = self.magnet_tip_position - object_pos
        distance = float(np.linalg.norm(rel))
        rel_vel = self.magnet_tip_velocity - self.object_velocity
        if self.rel_vel_filter_tau_s > 0.0:
            t_sim = self.get_clock().now().nanoseconds * 1e-9     # the wall clock runs 4x at RTF 0.25
            if self._rel_vel_f is None:
                self._rel_vel_f = rel_vel.copy()
            else:
                a = min(1.0, max(0.0, t_sim - self._rel_vel_t) / self.rel_vel_filter_tau_s)
                self._rel_vel_f = self._rel_vel_f + a * (rel_vel - self._rel_vel_f)
            self._rel_vel_t = t_sim
            rel_vel = self._rel_vel_f
        relative_speed = float(np.linalg.norm(rel_vel))
        self.last_distance = distance
        self.last_relative_speed = relative_speed

        close_enough = distance <= self.attach_radius
        slow_enough = relative_speed <= self.attach_speed_threshold
        can_attach_now = self.magnet_on and close_enough and slow_enough

        if self.magnet_on and not self.attached:
            if can_attach_now:
                if self.attach_condition_start is None:
                    self.attach_condition_start = now
                dwell = now - self.attach_condition_start
                if dwell >= self.attach_dwell_time_s:
                    if self.attachment_mode == 'fixed_joint':
                        self.command_attach_once(
                            f'd={distance:.3f} m, rel_speed={relative_speed:.3f} m/s, dwell={dwell:.2f} s'
                        )
                    elif self.attachment_mode == 'kinematic':
                        self.attached = True
                        self.attach_command_sent = True
                        self.detach_command_sent = False
                        self.get_logger().info(
                            f'KINEMATIC ATTACH: d={distance:.3f} m, rel_speed={relative_speed:.3f} m/s, dwell={dwell:.2f} s'
                        )
                    else:
                        self.get_logger().warn(f'Unknown attachment_mode={self.attachment_mode!r}; not attaching')
            else:
                self.attach_condition_start = None

        if not self.magnet_on and self.attached and self.detach_when_magnet_off:
            self.command_detach_once('magnet OFF while attached')

        self.update_kinematic_attachment(now)

        dwell_text = 0.0 if self.attach_condition_start is None else now - self.attach_condition_start
        state = (
            f'magnet_on={self.magnet_on} attached={self.attached} '
            f'mode={self.attachment_mode} backend={self.command_backend} '
            f'distance={distance:.3f}m radius={self.attach_radius:.3f}m '
            f'rel_speed={relative_speed:.3f}m/s threshold={self.attach_speed_threshold:.3f}m/s '
            f'close={close_enough} slow={slow_enough} dwell={dwell_text:.2f}/{self.attach_dwell_time_s:.2f}s'
        )
        self.publish_state(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MagnetAttachmentManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
