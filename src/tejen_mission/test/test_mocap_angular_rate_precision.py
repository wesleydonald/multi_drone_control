"""Small physical body rates must survive the shared IRL mocap parser."""

from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time

from tejen_mission import motion_capture_publisher_irl as mocap
from tejen_mission.m2_irl_mocap_router import RigidBodyParser


def test_slow_body_rotation_preserves_rate_through_m1_and_m2(monkeypatch):
    now = 100.0
    monkeypatch.setattr(mocap.time, "time", lambda: now)
    m1_parser = mocap.ParseData()
    m2_parser = RigidBodyParser()
    omega = np.array([-0.021, 0.032, -0.043])  # rad/s, below old rounding bins
    magnitude = np.linalg.norm(omega)
    dt = 0.01

    for step in range(81):
        now = 100.0 + (step + 1) * dt
        half_angle = magnitude * step * dt / 2.0
        xyz = omega / magnitude * np.sin(half_angle)
        quaternion = [*xyz, np.cos(half_angle)]  # UDP input is x/y/z/w
        packet = "12|1.0,-2.0,0.7|" + ",".join(map(str, quaternion))
        m1_sample = m1_parser.parse_packet(packet)
        m2_sample = m2_parser.parse(packet)
        assert m1_sample is not None and m2_sample is not None

    # Keep the existing filter and quaternion finite-difference convention.
    np.testing.assert_allclose(m1_sample.angular_velocity, omega, atol=1e-6)
    np.testing.assert_allclose(m2_sample.angular_velocity, omega, atol=1e-6)

    clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    publisher = SimpleNamespace(get_clock=lambda: clock)
    message = mocap.MotionCapturePublisher.create_motion_capture_state_msg(
        publisher, m1_sample
    )
    np.testing.assert_allclose(
        [message.twist.angular.x, message.twist.angular.y, message.twist.angular.z],
        omega,
        atol=1e-6,
    )
