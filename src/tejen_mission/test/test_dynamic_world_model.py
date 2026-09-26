import math

import numpy as np

from tejen_mission.cooperative_trajectory import (
    AttachmentPointTrajectory,
    OffsetPointTrajectory,
    SinusoidalLineTrajectory,
    StaticRigidBodyTrajectory,
    TrajectoryState,
    basket_attachment_positions,
    rigid_body_offset_state,
)
from tejen_mission.rendezvous import (
    estimate_rendezvous_time,
    min_time_double_integrator_1d,
    min_time_double_integrator_3d,
)
from tejen_mission.time_indexed_hulls import (
    AxisAlignedEnvelope,
    build_sinusoidal_hulls,
    maximum_centreline_containment_violation,
    uniform_intervals,
)


def test_agreed_sinusoidal_obstacle_state_and_derivatives() -> None:
    trajectory = SinusoidalLineTrajectory()
    state0 = trajectory.state(0.0)
    np.testing.assert_allclose(state0.position, [1.0, 0.70, 1.50], atol=1e-12)
    np.testing.assert_allclose(state0.velocity, [0.0, 0.28, 0.0], atol=1e-12)
    np.testing.assert_allclose(state0.acceleration, [0.0, 0.0, 0.0], atol=1e-12)

    t = 2.3
    dt = 1e-5
    p_minus = trajectory.state(t - dt).position
    p_plus = trajectory.state(t + dt).position
    numerical_velocity = (p_plus - p_minus) / (2.0 * dt)
    np.testing.assert_allclose(trajectory.state(t).velocity, numerical_velocity, atol=1e-8)


def test_exact_sinusoidal_extremum_inside_interval() -> None:
    trajectory = SinusoidalLineTrajectory()
    t_peak = math.pi / (2.0 * trajectory.omega)
    y_min, y_max = trajectory.y_extrema(t_peak - 1.0, t_peak + 1.0)
    assert abs(y_max - (trajectory.y_center + trajectory.y_amplitude)) < 1e-12
    assert y_min < y_max


def test_time_indexed_hulls_contain_entire_centreline() -> None:
    trajectory = SinusoidalLineTrajectory()
    envelope = AxisAlignedEnvelope(np.array([0.105, 0.105, 0.060]), 0.0)
    edges = uniform_intervals(0.0, 8.0, 4)
    hulls = build_sinusoidal_hulls(trajectory, envelope, edges)
    assert len(hulls) == 4
    assert maximum_centreline_containment_violation(trajectory, hulls) <= 1e-12

    for hull in hulls:
        assert hull.vertices.shape == (8, 3)
        for t in np.linspace(hull.t_start, hull.t_end, 301):
            assert hull.contains(trajectory.state(float(t)).position)


def test_tracking_margin_is_separate_from_physical_size() -> None:
    envelope = AxisAlignedEnvelope(np.array([0.10, 0.11, 0.06]), 0.03)
    np.testing.assert_allclose(envelope.effective_half_extents_m, [0.13, 0.14, 0.09])


def test_twelve_attachment_points_are_equispaced_on_50cm_basket() -> None:
    basket = StaticRigidBodyTrajectory(np.array([2.0, 2.0, 1.2]))
    points = basket_attachment_positions(basket.state(0.0))
    assert points.shape == (12, 3)
    radial = points[:, :2] - np.array([2.0, 2.0])
    np.testing.assert_allclose(np.linalg.norm(radial, axis=1), 0.25, atol=1e-12)
    np.testing.assert_allclose(points[:, 2], 1.2, atol=1e-12)

    angles = np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))
    np.testing.assert_allclose(np.diff(angles), 2.0 * math.pi / 12.0, atol=1e-12)


def test_known_attachment_id_and_existing_30cm_approach_height() -> None:
    basket = StaticRigidBodyTrajectory(np.array([2.0, 2.0, 1.2]))
    attachment = AttachmentPointTrajectory(basket, attachment_id=0)
    approach = OffsetPointTrajectory(attachment, np.array([0.0, 0.0, 0.30]))
    np.testing.assert_allclose(attachment.state(0.0).position, [2.25, 2.0, 1.2], atol=1e-12)
    np.testing.assert_allclose(approach.state(0.0).position, [2.25, 2.0, 1.5], atol=1e-12)


def test_yaw_rotates_attachment_and_generates_velocity() -> None:
    class YawingBasket:
        def state(self, t: float):
            from tejen_mission.cooperative_trajectory import RigidBodyState
            return RigidBodyState(
                position=np.array([0.0, 0.0, 1.2]),
                velocity=np.zeros(3),
                acceleration=np.zeros(3),
                yaw=t,
                yaw_rate=1.0,
                yaw_acceleration=0.0,
            )

    attachment = AttachmentPointTrajectory(YawingBasket(), attachment_id=0)
    state = attachment.state(math.pi / 2.0)
    np.testing.assert_allclose(state.position, [0.0, 0.25, 1.2], atol=1e-12)
    np.testing.assert_allclose(state.velocity, [-0.25, 0.0, 0.0], atol=1e-12)


def test_rigid_body_offset_state_orbits_with_body_yaw_rate() -> None:
    from tejen_mission.cooperative_trajectory import RigidBodyState

    body = RigidBodyState(
        position=np.array([2.0, 3.0, 1.5]),
        velocity=np.array([0.10, -0.20, 0.0]),
        acceleration=np.zeros(3),
        yaw=math.pi / 2.0,
        yaw_rate=0.20,
        yaw_acceleration=0.0,
    )
    state = rigid_body_offset_state(body, np.array([0.60, 0.0, 0.35]))

    np.testing.assert_allclose(state.position, [2.0, 3.60, 1.85], atol=1e-12)
    # Rz(pi/2)[0.60,0,0.35] = [0,0.60,0.35], so omega x r = [-0.12,0,0].
    np.testing.assert_allclose(state.velocity, [-0.02, -0.20, 0.0], atol=1e-12)
    np.testing.assert_allclose(state.acceleration, [0.0, -0.024, 0.0], atol=1e-12)


def test_rmader_double_integrator_3d_is_maximum_axis_time() -> None:
    p0 = np.array([0.0, 0.0, 1.5])
    v0 = np.zeros(3)
    pf = np.array([2.25, 2.0, 1.5])
    vf = np.zeros(3)
    vmax = np.ones(3)
    amax = np.full(3, 1.5)

    axis_times = [
        min_time_double_integrator_1d(p0[i], v0[i], pf[i], vf[i], vmax[i], amax[i])
        for i in range(3)
    ]
    assert abs(min_time_double_integrator_3d(p0, v0, pf, vf, vmax, amax) - max(axis_times)) < 1e-12
    assert abs(max(axis_times) - 2.9166666666666665) < 1e-12


def test_static_rendezvous_uses_rmader_close_goal_allocation_factor() -> None:
    start = TrajectoryState(np.array([0.0, 0.0, 1.5]), np.zeros(3), np.zeros(3))
    basket = StaticRigidBodyTrajectory(np.array([2.0, 2.0, 1.2]))
    approach = OffsetPointTrajectory(
        AttachmentPointTrajectory(basket, attachment_id=0),
        np.array([0.0, 0.0, 0.30]),
    )
    estimate = estimate_rendezvous_time(
        start,
        approach,
        v_max=np.ones(3),
        a_max=np.full(3, 1.5),
    )
    assert estimate.converged
    assert estimate.allocation_factor == 2.5
    assert abs(estimate.duration_s - 2.5 * 2.9166666666666665) < 1e-12
    np.testing.assert_allclose(estimate.goal_state.velocity, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(estimate.goal_state.acceleration, np.zeros(3), atol=1e-12)
