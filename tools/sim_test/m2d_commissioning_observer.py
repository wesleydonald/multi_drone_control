#!/usr/bin/env python3
"""Persistent functional-state observer for M2D commissioning.

This node is external commissioning infrastructure. It intentionally uses wall time
for its own status snapshots so it remains responsive while Gazebo is paused or the
simulation runs below real time. Mission/controller semantics remain simulation-time
owned by the runtime nodes themselves.

The observer deliberately does *not* use ROS graph discovery as readiness authority.
It records only end-to-end state published by the component that owns each contract:
fixture truth, planner phase, backend runtime status, controller ARM/TAKEOFF feedback,
and the M2C/M2D aggregate states.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import tempfile
import time
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String

DRONE_IDS = range(4)


def latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
        os.replace(tmp_name, path)
    finally:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass


class M2DCommissioningObserver(Node):
    def __init__(self, status_file: Path, env_file: Path, period_s: float) -> None:
        super().__init__("m2d_commissioning_observer")
        self.status_file = status_file
        self.env_file = env_file
        self.start_monotonic = time.monotonic()
        self.messages: Dict[str, Dict[str, Any]] = {}
        self.latest_m2c_status: Dict[str, Any] = {}
        self.latest_m2d_status: Dict[str, Any] = {}
        self.latest_assignment: Dict[str, Any] = {}
        self.m2c_ready = False
        self.m2c_ready_latched = False
        self.clock_sim_ns = None

        self.create_subscription(Clock, "/clock", self._clock_callback, 10)
        self.create_subscription(String, "/m2c/status", self._m2c_status, 10)
        self.create_subscription(Bool, "/m2c/ready", self._m2c_ready, 10)
        self.create_subscription(String, "/m2c/assignment", self._assignment, latched_qos())
        self.create_subscription(String, "/m2d/status", self._m2d_status, latched_qos())

        for drone_id in DRONE_IDS:
            ns = f"/drone_{drone_id}"
            self.create_subscription(
                Bool,
                f"{ns}/magnet/joint_detached_truth",
                lambda msg, i=drone_id: self._mark(
                    f"drone_{i}.joint_detached", bool(msg.data), snapshot_on_change=True
                ),
                10,
            )
            self.create_subscription(
                String,
                f"{ns}/join_planner/phase",
                lambda msg, i=drone_id: self._mark(
                    f"drone_{i}.phase", str(msg.data), snapshot_on_change=True
                ),
                latched_qos(),
            )
            self.create_subscription(
                String,
                f"{ns}/dynamic_planner/transfer_status",
                lambda msg, i=drone_id: self._mark(f"drone_{i}.backend_status", str(msg.data)),
                10,
            )
            self.create_subscription(
                Bool,
                f"{ns}/arming_state_feedback",
                lambda msg, i=drone_id: self._mark(
                    f"drone_{i}.armed", bool(msg.data), snapshot_on_change=True
                ),
                10,
            )
            self.create_subscription(
                Bool,
                f"{ns}/takeoff_state_feedback",
                lambda msg, i=drone_id: self._mark(
                    f"drone_{i}.takeoff", bool(msg.data), snapshot_on_change=True
                ),
                10,
            )

        self.timer = self.create_timer(max(0.10, period_s), self._snapshot)
        self._snapshot()
        self.get_logger().info(
            f"M2D commissioning observer started: status={self.status_file}, env={self.env_file}"
        )

    def _mark(self, key: str, value: Any, *, snapshot_on_change: bool = False) -> None:
        previous = self.messages.get(key)
        changed = previous is None or previous.get("value") != value
        self.messages[key] = {
            "seen": True,
            "value": value,
            "wall_monotonic_s": time.monotonic(),
        }
        if snapshot_on_change and changed:
            self._snapshot()

    def _clock_callback(self, msg: Clock) -> None:
        self.clock_sim_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        self._mark("clock", self.clock_sim_ns)

    def _m2c_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.latest_m2c_status = payload
        except json.JSONDecodeError:
            pass
        self._mark("m2c.status", msg.data)

    def _m2c_ready(self, msg: Bool) -> None:
        self.m2c_ready = bool(msg.data)
        if self.m2c_ready:
            self.m2c_ready_latched = True
        self._mark("m2c.ready", self.m2c_ready)

    def _assignment(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.latest_assignment = payload
        except json.JSONDecodeError:
            pass
        self._mark("m2c.assignment", msg.data)

    def _m2d_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.latest_m2d_status = payload
        except json.JSONDecodeError:
            pass
        self._mark("m2d.status", msg.data)

    def _message_snapshot(self, key: str) -> Dict[str, Any]:
        item = self.messages.get(key)
        if item is None:
            return {"seen": False, "value": None, "age_wall_s": None}
        return {
            "seen": True,
            "value": item["value"],
            "age_wall_s": max(0.0, time.monotonic() - float(item["wall_monotonic_s"])),
        }

    def _snapshot(self) -> None:
        now_unix_s = int(time.time())
        drones: Dict[str, Any] = {}
        env: Dict[str, Any] = {
            "observer_ready": True,
            "observer_wall_unix_s": now_unix_s,
            "observer_uptime_s": round(time.monotonic() - self.start_monotonic, 3),
            "clock_seen": self._message_snapshot("clock")["seen"],
            "clock_sim_ns": self.clock_sim_ns if self.clock_sim_ns is not None else "",
            "m2c_status_seen": self._message_snapshot("m2c.status")["seen"],
            "m2c_ready_seen": self._message_snapshot("m2c.ready")["seen"],
            "m2c_ready": self.m2c_ready,
            "m2c_ready_latched": self.m2c_ready_latched,
            "m2c_assignment_seen": self._message_snapshot("m2c.assignment")["seen"],
            "m2d_status_seen": self._message_snapshot("m2d.status")["seen"],
        }

        for key in (
            "state",
            "joint_detached_count",
            "ground_settled",
            "disarmed_controller_count",
            "assignment_frozen",
            "candidate_count",
        ):
            env[f"m2c_{key}"] = self.latest_m2c_status.get(key, "")
        env["m2d_state"] = self.latest_m2d_status.get("state", "")
        env["m2d_abort_reason"] = self.latest_m2d_status.get("abort_reason", "") or ""

        for drone_id in DRONE_IDS:
            name = f"drone_{drone_id}"
            backend_status = self._message_snapshot(f"{name}.backend_status")
            phase = self._message_snapshot(f"{name}.phase")
            joint = self._message_snapshot(f"{name}.joint_detached")
            armed = self._message_snapshot(f"{name}.armed")
            takeoff = self._message_snapshot(f"{name}.takeoff")

            drone = {
                "joint_detached": joint,
                "phase": phase,
                "backend_status": backend_status,
                "armed": armed,
                "takeoff": takeoff,
            }
            drones[name] = drone

            prefix = f"{name}_"
            env[f"{prefix}joint_detached_seen"] = joint["seen"]
            env[f"{prefix}joint_detached"] = joint["value"] if joint["seen"] else False
            env[f"{prefix}phase_seen"] = phase["seen"]
            env[f"{prefix}phase"] = phase["value"] if phase["seen"] else ""
            env[f"{prefix}backend_status_seen"] = backend_status["seen"]
            env[f"{prefix}armed_seen"] = armed["seen"]
            env[f"{prefix}armed"] = armed["value"] if armed["seen"] else False
            env[f"{prefix}takeoff_seen"] = takeoff["seen"]
            env[f"{prefix}takeoff"] = takeoff["value"] if takeoff["seen"] else False

        payload = {
            "observer": {
                "ready": True,
                "wall_unix_s": now_unix_s,
                "uptime_s": env["observer_uptime_s"],
                "pid": os.getpid(),
            },
            "clock": self._message_snapshot("clock"),
            "m2c": {
                "status_seen": env["m2c_status_seen"],
                "status": self.latest_m2c_status,
                "ready_seen": env["m2c_ready_seen"],
                "ready": self.m2c_ready,
                "ready_latched": self.m2c_ready_latched,
                "assignment_seen": env["m2c_assignment_seen"],
                "assignment": self.latest_assignment,
            },
            "m2d": {
                "status_seen": env["m2d_status_seen"],
                "status": self.latest_m2d_status,
            },
            "drones": drones,
        }
        _atomic_text(self.status_file, json.dumps(payload, indent=2, sort_keys=True) + "\n")

        env_lines = []
        for key in sorted(env):
            value = env[key]
            if isinstance(value, bool):
                rendered = "true" if value else "false"
            elif value is None:
                rendered = ""
            else:
                rendered = str(value)
            env_lines.append(f"{key}={shlex.quote(rendered)}")
        _atomic_text(self.env_file, "\n".join(env_lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--status-file", required=True)
    parser.add_argument("--env-file", required=True)
    parser.add_argument("--period-s", type=float, default=0.50)
    args = parser.parse_args()

    rclpy.init()
    node = M2DCommissioningObserver(Path(args.status_file), Path(args.env_file), args.period_s)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
