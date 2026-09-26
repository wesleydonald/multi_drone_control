from __future__ import annotations

import importlib
import math
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "tools" / "sim_test" / "run_m2d_sequential_attachment.sh"


def _spawn_module():
    try:
        return importlib.import_module("tejen_mission.m2c_ground_spawn")
    except ModuleNotFoundError:
        pytest.fail("M2D hard-geometry contract: m2c_ground_spawn is unavailable")


def test_nominal_stage_layout_remains_the_commissioned_default():
    spawn = _spawn_module()
    layout = spawn.resolve_stage_layout("nominal", ring_yaw_deg=20.0)

    assert layout.name == "nominal"
    assert layout.stage_radius_m == tuple(float(v) for v in spawn.DEFAULT_STAGE_RADIUS_M)
    assert layout.stage_angle_deg == tuple(float(v) for v in spawn.DEFAULT_STAGE_ANGLE_DEG)
    assert layout.body_yaw_deg == tuple(float(v) for v in spawn.DEFAULT_BODY_YAW_DEG)


def test_hard_line_layout_puts_three_drones_on_one_ring_radial_spoke_outer_to_inner():
    spawn = _spawn_module()
    ring_yaw_deg = 20.0
    layout = spawn.resolve_stage_layout("hard_line", ring_yaw_deg=ring_yaw_deg)

    assert layout.name == "hard_line"
    assert layout.stage_angle_deg[:3] == (ring_yaw_deg, ring_yaw_deg, ring_yaw_deg)
    assert layout.stage_radius_m[0] > layout.stage_radius_m[1] > layout.stage_radius_m[2]
    assert layout.stage_radius_m[:3] == pytest.approx((2.25, 1.60, 0.95))
    assert layout.stage_angle_deg[3] == pytest.approx(ring_yaw_deg - 95.0)

    body_xy = []
    for i in range(4):
        geom = spawn.grounded_vehicle_geometry(
            i,
            stage_radius_m=layout.stage_radius_m[i],
            stage_angle_rad=math.radians(layout.stage_angle_deg[i]),
            body_yaw_rad=math.radians(layout.body_yaw_deg[i]),
        )
        body_xy.append(np.asarray(geom.body_position_world[:2], dtype=float))

    # Drones 0/1/2 are exactly collinear with the ring centre and share the same
    # outward direction, while drone 3 is genuinely off-axis.
    line_direction = body_xy[2] / np.linalg.norm(body_xy[2])
    for i in (0, 1, 2):
        point = body_xy[i]
        cross_z = line_direction[0] * point[1] - line_direction[1] * point[0]
        assert abs(float(cross_z)) < 1e-12
        assert float(np.dot(line_direction, point)) > 0.0
    off_axis_cross = line_direction[0] * body_xy[3][1] - line_direction[1] * body_xy[3][0]
    assert abs(float(off_axis_cross)) > 0.5


def test_hard_line_ground_geometry_starts_collision_separated_before_planning():
    spawn = _spawn_module()
    layout = spawn.resolve_stage_layout("hard_line", ring_yaw_deg=20.0)

    geometries = [
        spawn.grounded_vehicle_geometry(
            i,
            stage_radius_m=layout.stage_radius_m[i],
            stage_angle_rad=math.radians(layout.stage_angle_deg[i]),
            body_yaw_rad=math.radians(layout.body_yaw_deg[i]),
        )
        for i in range(4)
    ]

    body_xy = [np.asarray(g.body_position_world[:2], dtype=float) for g in geometries]
    magnet_xy = [np.asarray(g.magnet_center_world[:2], dtype=float) for g in geometries]

    # Consecutive line drones have 0.65 m body-centre spacing.  The source X3
    # body collision is 0.12 x 0.05 m and the magnet radius is 0.025 m, so the
    # closest outward magnet-to-next-body centre gap (~0.202 m) is comfortably
    # larger than the corresponding collision radii.
    assert np.linalg.norm(body_xy[0] - body_xy[1]) == pytest.approx(0.65, abs=1e-12)
    assert np.linalg.norm(body_xy[1] - body_xy[2]) == pytest.approx(0.65, abs=1e-12)
    assert np.linalg.norm(magnet_xy[1] - body_xy[0]) > 0.20
    assert np.linalg.norm(magnet_xy[2] - body_xy[1]) > 0.20

    # The inner line vehicle still starts well outside the ~0.28 m basket
    # footprint used by M2C/M2D commissioning.
    assert np.linalg.norm(body_xy[2]) > 0.90

    for i in range(4):
        for j in range(i + 1, 4):
            assert np.linalg.norm(body_xy[i] - body_xy[j]) > 0.60


def test_m2d_runner_selects_hard_line_only_when_explicitly_requested():
    runner = RUNNER.read_text(encoding="utf-8")

    assert 'SPAWN_SCENARIO="${M2D_SPAWN_SCENARIO:-nominal}"' in runner
    assert "nominal|hard_line)" in runner
    assert '--stage-layout "$SPAWN_SCENARIO"' in runner
    assert 'echo "Scenario:   $SPAWN_SCENARIO"' in runner
    assert "spawn_scenario=$SPAWN_SCENARIO" in runner
