#!/usr/bin/env python3
"""Bounded, simulation-only mission supervisor for repeatable system tests."""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from interfaces.msg import MotionCaptureState
from interfaces.srv import SetArming
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class SupervisorAction(str, Enum):
    REQUEST_ARM = "REQUEST_ARM"
    PUBLISH_TAKEOFF = "PUBLISH_TAKEOFF"
    PUBLISH_LAND = "PUBLISH_LAND"
    REQUEST_DISARM = "REQUEST_DISARM"
    FINISH = "FINISH"


class SupervisorState(str, Enum):
    WAITING_READY = "WAITING_READY"
    ARMING = "ARMING"
    WAITING_TAKEOFF = "WAITING_TAKEOFF"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    DONE = "DONE"


@dataclass(frozen=True)
class SupervisorConfig:
    control_mode: str = "manual"
    startup_timeout_s: float = 60.0
    manual_start_timeout_s: float = 300.0
    arm_timeout_s: float = 5.0
    mission_timeout_s: float = 120.0
    success_phase: str = "ATTACH_READY"
    success_dwell_s: float = 2.0
    require_handoff_ready: bool = True
    failure_phase: str = "LANDED_DISARMED"
    landing_request_phase: str = ""
    takeoff_retry_s: float = 1.0
    takeoff_max_publications: int = 3
    shutdown_timeout_s: float = 3.0

    def validate(self) -> None:
        if self.control_mode not in {"automatic", "manual"}:
            raise ValueError("control_mode must be automatic or manual")
        for name in (
            "startup_timeout_s",
            "manual_start_timeout_s",
            "arm_timeout_s",
            "mission_timeout_s",
            "success_dwell_s",
            "takeoff_retry_s",
            "shutdown_timeout_s",
        ):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.takeoff_max_publications < 1:
            raise ValueError("takeoff_max_publications must be at least one")


@dataclass(frozen=True)
class SupervisorSnapshot:
    inputs_ready: bool
    phase: Optional[str]
    handoff_ready: bool
    armed: Optional[bool]


@dataclass(frozen=True)
class SupervisorDecision:
    actions: Tuple[SupervisorAction, ...] = ()
    outcome: Optional[str] = None
    category: Optional[str] = None
    reason: Optional[str] = None
    exit_code: Optional[int] = None


class SupervisorCore:
    """Pure state machine; ROS callbacks only provide timestamped observations."""

    def __init__(self, config: SupervisorConfig, start_time: float = 0.0):
        config.validate()
        self.config = config
        self.state = SupervisorState.WAITING_READY
        self.start_time = float(start_time)
        self.state_entry_time = float(start_time)
        self.mission_start_time: Optional[float] = None
        self.success_start_time: Optional[float] = None
        self.last_takeoff_publish_time: Optional[float] = None
        self.takeoff_publications = 0
        self.landing_requested = False
        self.armed_once = False
        self.arm_result: Optional[bool] = None
        self.final_outcome: Optional[str] = None
        self.final_category: Optional[str] = None
        self.final_reason: Optional[str] = None
        self.final_exit_code: Optional[int] = None
        self.events: list[dict[str, object]] = []

    def _event(self, now: float, name: str, **details: object) -> None:
        self.events.append({"elapsed_s": max(0.0, now - self.start_time), "event": name, **details})

    def _transition(self, now: float, state: SupervisorState) -> None:
        self.state = state
        self.state_entry_time = now
        self._event(now, "state", state=state.value)

    def _stop(
        self, now: float, outcome: str, category: str, reason: str, exit_code: int
    ) -> SupervisorDecision:
        self.final_outcome = outcome
        self.final_category = category
        self.final_reason = reason
        self.final_exit_code = exit_code
        self._event(now, "terminal", outcome=outcome, category=category, reason=reason)
        self._transition(now, SupervisorState.STOPPING)
        return SupervisorDecision(
            actions=(SupervisorAction.REQUEST_DISARM,),
            outcome=outcome,
            category=category,
            reason=reason,
            exit_code=exit_code,
        )

    def _publish_takeoff(self, now: float) -> SupervisorDecision:
        self.takeoff_publications += 1
        self.last_takeoff_publish_time = now
        self._event(now, "takeoff_publish", count=self.takeoff_publications)
        return SupervisorDecision(actions=(SupervisorAction.PUBLISH_TAKEOFF,))

    def set_arm_result(self, now: float, accepted: bool) -> None:
        self.arm_result = bool(accepted)
        self._event(now, "arm_response", accepted=bool(accepted))

    def step(self, now: float, snapshot: SupervisorSnapshot) -> SupervisorDecision:
        now = float(now)
        if snapshot.armed is True:
            self.armed_once = True

        if self.state == SupervisorState.WAITING_READY:
            if now - self.start_time >= self.config.startup_timeout_s:
                return self._stop(now, "FAIL", "infrastructure", "STARTUP_TIMEOUT", 2)
            if not snapshot.inputs_ready:
                return SupervisorDecision()
            if self.config.control_mode == "automatic":
                self._transition(now, SupervisorState.ARMING)
                self._event(now, "arm_request")
                return SupervisorDecision(actions=(SupervisorAction.REQUEST_ARM,))
            self._transition(now, SupervisorState.WAITING_TAKEOFF)
            return SupervisorDecision()

        if self.state == SupervisorState.ARMING:
            if not snapshot.inputs_ready:
                return self._stop(now, "FAIL", "infrastructure", "READINESS_LOST", 2)
            if self.arm_result is False:
                return self._stop(now, "FAIL", "infrastructure", "ARM_REJECTED", 2)
            if snapshot.armed is True:
                self._transition(now, SupervisorState.WAITING_TAKEOFF)
                return self._publish_takeoff(now)
            if now - self.state_entry_time >= self.config.arm_timeout_s:
                return self._stop(now, "FAIL", "infrastructure", "ARM_TIMEOUT", 2)
            return SupervisorDecision()

        if self.state == SupervisorState.WAITING_TAKEOFF:
            if not snapshot.inputs_ready:
                return self._stop(now, "FAIL", "infrastructure", "READINESS_LOST", 2)
            if snapshot.phase == self.config.failure_phase:
                return self._stop(now, "FAIL", "mission", "EARLY_TERMINAL_DISARM", 1)
            # any phase past WAIT_FOR_TAKEOFF counts: a drone handed over in the air
            # confirms its takeoff at once and can leave TAKEOFF within one supervisor tick
            # (multi_drone_control R0637: TAKEOFF_NOT_OBSERVED -> disarm mid-air)
            if snapshot.phase not in (None, "", "WAIT_FOR_TAKEOFF", self.config.failure_phase):
                self.mission_start_time = now
                self._event(now, "mission_clock_started")
                self._transition(now, SupervisorState.RUNNING)
                return SupervisorDecision()
            if self.config.control_mode == "manual":
                if self.armed_once and snapshot.armed is False:
                    return self._stop(now, "FAIL", "mission", "DISARMED_BEFORE_TAKEOFF", 1)
                if now - self.state_entry_time >= self.config.manual_start_timeout_s:
                    return self._stop(now, "FAIL", "infrastructure", "MANUAL_START_TIMEOUT", 2)
                return SupervisorDecision()
            if snapshot.armed is False:
                return self._stop(now, "FAIL", "mission", "ARMING_LOST_BEFORE_TAKEOFF", 1)
            if snapshot.armed is True:
                since_publish = (
                    float("inf")
                    if self.last_takeoff_publish_time is None
                    else now - self.last_takeoff_publish_time
                )
                if since_publish >= self.config.takeoff_retry_s:
                    if self.takeoff_publications < self.config.takeoff_max_publications:
                        return self._publish_takeoff(now)
                    return self._stop(now, "FAIL", "mission", "TAKEOFF_NOT_OBSERVED", 1)
            return SupervisorDecision()

        if self.state == SupervisorState.RUNNING:
            if not snapshot.inputs_ready:
                return self._stop(now, "FAIL", "infrastructure", "REQUIRED_INPUT_STALE", 2)

            if (
                self.config.landing_request_phase
                and not self.landing_requested
                and snapshot.phase == self.config.landing_request_phase
            ):
                self.landing_requested = True
                self._event(now, "landing_request", phase=snapshot.phase)
                return SupervisorDecision(actions=(SupervisorAction.PUBLISH_LAND,))

            success_condition = (
                snapshot.phase == self.config.success_phase
                and (
                    snapshot.handoff_ready
                    or not self.config.require_handoff_ready
                )
            )
            if success_condition:
                if self.success_start_time is None:
                    self.success_start_time = now
                    self._event(now, "success_dwell_started")
                    return SupervisorDecision()
                if now - self.success_start_time >= self.config.success_dwell_s:
                    return self._stop(now, "PASS", "mission", "SUCCESS_DWELL_REACHED", 0)
                return SupervisorDecision()
            if self.success_start_time is not None:
                self._event(now, "success_dwell_reset")
                self.success_start_time = None

            if snapshot.phase == self.config.failure_phase:
                return self._stop(now, "FAIL", "mission", "EARLY_TERMINAL_DISARM", 1)
            if snapshot.armed is False:
                return self._stop(now, "FAIL", "mission", "PREMATURE_DISARM", 1)
            if (
                self.mission_start_time is not None
                and now - self.mission_start_time >= self.config.mission_timeout_s
            ):
                return self._stop(now, "FAIL", "mission", "MISSION_TIMEOUT", 1)
            return SupervisorDecision()

        if self.state == SupervisorState.STOPPING:
            if snapshot.armed is False or now - self.state_entry_time >= self.config.shutdown_timeout_s:
                self._transition(now, SupervisorState.DONE)
                return SupervisorDecision(
                    actions=(SupervisorAction.FINISH,),
                    outcome=self.final_outcome,
                    category=self.final_category,
                    reason=self.final_reason,
                    exit_code=self.final_exit_code,
                )
            return SupervisorDecision()

        return SupervisorDecision(
            outcome=self.final_outcome,
            category=self.final_category,
            reason=self.final_reason,
            exit_code=self.final_exit_code,
        )


class SimulationTestSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("simulation_test_supervisor")
        config = SupervisorConfig(
            control_mode=str(self.declare_parameter("control_mode", "manual").value),
            startup_timeout_s=float(self.declare_parameter("startup_timeout_s", 60.0).value),
            manual_start_timeout_s=float(self.declare_parameter("manual_start_timeout_s", 300.0).value),
            arm_timeout_s=float(self.declare_parameter("arm_timeout_s", 5.0).value),
            mission_timeout_s=float(self.declare_parameter("mission_timeout_s", 120.0).value),
            success_phase=str(self.declare_parameter("success_phase", "ATTACH_READY").value),
            success_dwell_s=float(self.declare_parameter("success_dwell_s", 2.0).value),
            require_handoff_ready=bool(
                self.declare_parameter("require_handoff_ready", True).value
            ),
            failure_phase=str(self.declare_parameter("failure_phase", "LANDED_DISARMED").value),
            landing_request_phase=str(
                self.declare_parameter("landing_request_phase", "").value
            ).strip(),
            takeoff_retry_s=float(self.declare_parameter("takeoff_retry_s", 1.0).value),
            takeoff_max_publications=int(self.declare_parameter("takeoff_max_publications", 3).value),
            shutdown_timeout_s=float(self.declare_parameter("shutdown_timeout_s", 3.0).value),
        )
        # optional start gate (multi_drone_control partner mode): hold the whole schedule,
        # startup timeout included, until a JSON String topic reports "running": true
        # (our load planner's /payload/trajectory_state once the ring flies its path)
        self.start_gate_topic = str(self.declare_parameter("start_gate_topic", "").value).strip()
        self.start_gate_open = not self.start_gate_topic
        if self.start_gate_topic:
            self.create_subscription(String, self.start_gate_topic, self._start_gate, 5)
        result_path_value = str(self.declare_parameter("result_path", "").value).strip()
        if not result_path_value:
            raise ValueError("result_path is required")
        self.result_path = Path(result_path_value)
        self.freshness_timeout_s = float(
            self.declare_parameter("freshness_timeout_s", 1.0).value
        )
        self.planner_status_timeout_s = float(
            self.declare_parameter("planner_status_timeout_s", 2.5).value
        )
        self.started_monotonic = time.monotonic()
        self.started_wall = time.time()
        self.core = SupervisorCore(config, self.started_monotonic)
        self.phase: Optional[str] = None
        self.handoff_ready = False
        self.armed: Optional[bool] = None
        self.last_motion: Optional[float] = None
        self.last_pendulum: Optional[float] = None
        self.last_payload: Optional[float] = None
        self.last_planner_status: Optional[float] = None
        self.arm_future = None
        self.disarm_future = None
        self.finished = False
        self.exit_code = 2

        phase_qos = QoSProfile(depth=1)
        phase_qos.reliability = ReliabilityPolicy.RELIABLE
        phase_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/join_planner/phase", self._phase, phase_qos)
        self.create_subscription(Bool, "/join_planner/handoff_ready", self._handoff, 5)
        self.create_subscription(Bool, "drone_arming_state_feedback", self._arming, 5)
        self.create_subscription(MotionCaptureState, "/motion_capture_state", self._motion, 5)
        self.create_subscription(MotionCaptureState, "/pendulum_swing_state", self._pendulum, 5)
        self.create_subscription(MotionCaptureState, "/payload_world_state", self._payload, 5)
        self.create_subscription(String, "/dynamic_planner/transfer_status", self._planner, 5)
        self.command_publisher = self.create_publisher(String, "drone_command", 5)
        self.land_publisher = self.create_publisher(Bool, "/join_planner/land_now", 5)
        self.arming_client = self.create_client(SetArming, "drone_arming_service")
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"Simulation supervisor started in {config.control_mode} mode; "
            f"mission timeout={config.mission_timeout_s:.1f}s"
        )

    def _phase(self, message: String) -> None:
        self.phase = message.data.strip()

    def _handoff(self, message: Bool) -> None:
        self.handoff_ready = bool(message.data)

    def _arming(self, message: Bool) -> None:
        self.armed = bool(message.data)

    def _motion(self, _: MotionCaptureState) -> None:
        self.last_motion = time.monotonic()

    def _pendulum(self, _: MotionCaptureState) -> None:
        self.last_pendulum = time.monotonic()

    def _payload(self, _: MotionCaptureState) -> None:
        self.last_payload = time.monotonic()

    def _planner(self, _: String) -> None:
        self.last_planner_status = time.monotonic()

    def _inputs_ready(self, now: float) -> bool:
        state_stamps = (self.last_motion, self.last_pendulum, self.last_payload)
        return bool(
            self.phase is not None
            and self.armed is not None
            and self.arming_client.service_is_ready()
            and self.command_publisher.get_subscription_count() > 0
            and all(
                stamp is not None and now - stamp <= self.freshness_timeout_s
                for stamp in state_stamps
            )
            and self.last_planner_status is not None
            and now - self.last_planner_status <= self.planner_status_timeout_s
        )

    def _readiness_diagnostics(self, now: float) -> dict[str, object]:
        def age(stamp: Optional[float]) -> Optional[float]:
            return None if stamp is None else max(0.0, now - stamp)

        return {
            "phase_received": self.phase is not None,
            "arming_feedback_received": self.armed is not None,
            "arming_service_ready": self.arming_client.service_is_ready(),
            "drone_command_subscribers": self.command_publisher.get_subscription_count(),
            "motion_age_s": age(self.last_motion),
            "pendulum_age_s": age(self.last_pendulum),
            "payload_age_s": age(self.last_payload),
            "planner_status_age_s": age(self.last_planner_status),
            "state_freshness_limit_s": self.freshness_timeout_s,
            "planner_status_freshness_limit_s": self.planner_status_timeout_s,
        }

    def _request_arm(self) -> None:
        if self.arm_future is not None:
            return
        request = SetArming.Request()
        request.arm = True
        self.arm_future = self.arming_client.call_async(request)
        self._arm_sent_at = time.monotonic()

    def _retry_arm_if_lost(self) -> None:
        # a request sent before the service is matched, or a response dropped under load,
        # never completes (multi_drone_control R0606: ARM_TIMEOUT). Arming is idempotent.
        if (self.arm_future is not None and not self.arm_future.done()
                and self.armed is not True
                and time.monotonic() - getattr(self, '_arm_sent_at', 0.0) > 2.0):
            self.get_logger().warn('Arming request unanswered after 2 s; re-sending')
            self.arm_future = None
            self._request_arm()

    def _request_disarm(self) -> None:
        if self.disarm_future is None and self.arming_client.service_is_ready():
            request = SetArming.Request()
            request.arm = False
            self.disarm_future = self.arming_client.call_async(request)
            return
        message = String()
        message.data = "DISARM"
        self.command_publisher.publish(message)

    def _publish_takeoff(self) -> None:
        message = String()
        message.data = "TAKEOFF"
        self.command_publisher.publish(message)

    def _publish_land(self) -> None:
        message = Bool()
        message.data = True
        self.land_publisher.publish(message)

    def _write_result(self, decision: SupervisorDecision) -> None:
        payload = {
            "schema_version": 1,
            "outcome": decision.outcome,
            "category": decision.category,
            "reason": decision.reason,
            "exit_code": decision.exit_code,
            "control_mode": self.core.config.control_mode,
            "phase": self.phase,
            "handoff_ready": self.handoff_ready,
            "armed": self.armed,
            "started_unix": self.started_wall,
            "finished_unix": time.time(),
            "mission_duration_s": (
                None
                if self.core.mission_start_time is None
                else max(0.0, time.monotonic() - self.core.mission_start_time)
            ),
            "takeoff_publications": self.core.takeoff_publications,
            "events": self.core.events,
            "final_readiness": self._readiness_diagnostics(time.monotonic()),
            "config": asdict(self.core.config),
        }
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.result_path.with_name(f".{self.result_path.name}.{os.getpid()}.tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        os.replace(temporary, self.result_path)

    def _start_gate(self, msg: String) -> None:
        if self.start_gate_open:
            return
        try:
            running = bool(json.loads(msg.data).get("running", False))
        except (ValueError, AttributeError):
            return
        if running:
            self.start_gate_open = True
            self.get_logger().info(f"Start gate open ({self.start_gate_topic} running)")

    def _tick(self) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if not self.start_gate_open:
            self.core.start_time = self.core.state_entry_time = now
            return
        self._retry_arm_if_lost()
        if self.core.state == SupervisorState.WAITING_READY and \
                now - getattr(self, '_ready_log_t', 0.0) >= 5.0:
            self._ready_log_t = now
            self.get_logger().info(f'waiting for inputs: {self._readiness_diagnostics(now)}')
        if self.arm_future is not None and self.arm_future.done() and self.core.arm_result is None:
            try:
                response = self.arm_future.result()
                self.core.set_arm_result(now, bool(response and response.success))
            except Exception as exc:  # service transport failure is infrastructure evidence
                self.get_logger().error(f"Arming service failed: {exc}")
                self.core.set_arm_result(now, False)
        snapshot = SupervisorSnapshot(
            inputs_ready=self._inputs_ready(now),
            phase=self.phase,
            handoff_ready=self.handoff_ready,
            armed=self.armed,
        )
        decision = self.core.step(now, snapshot)
        for action in decision.actions:
            if action == SupervisorAction.REQUEST_ARM:
                self._request_arm()
            elif action == SupervisorAction.PUBLISH_TAKEOFF:
                self._publish_takeoff()
            elif action == SupervisorAction.PUBLISH_LAND:
                self._publish_land()
            elif action == SupervisorAction.REQUEST_DISARM:
                self._request_disarm()
            elif action == SupervisorAction.FINISH:
                self._write_result(decision)
                self.exit_code = int(decision.exit_code if decision.exit_code is not None else 2)
                self.finished = True
                self.get_logger().info(
                    f"Supervisor finished: {decision.outcome} {decision.reason}"
                )


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[SimulationTestSupervisor] = None
    try:
        node = SimulationTestSupervisor()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
        return node.exit_code
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
