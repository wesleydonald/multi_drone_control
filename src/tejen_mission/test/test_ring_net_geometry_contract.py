import math

import numpy as np
import pytest

import tejen_mission.cooperative_trajectory as cooperative_trajectory


DEFAULT_PLATE_COUNT = 12
DEFAULT_PITCH_DIAMETER_M = 0.50
DEFAULT_PLATE_DIAMETER_M = 0.06
DEFAULT_COLLISION_OUTER_DIAMETER_M = 0.56
DEFAULT_COLLISION_TOP_OFFSET_M = 0.0
DEFAULT_COLLISION_BOTTOM_OFFSET_M = -0.27


def _ring_geometry_cls():
    cls = getattr(cooperative_trajectory, "RingNetGeometry", None)
    assert cls is not None, (
        "RingNetGeometry is the agreed pure geometry contract for the 50 cm pitch-circle "
        "ring. Step 3 must add it to cooperative_trajectory.py."
    )
    return cls


def _default_geometry():
    return _ring_geometry_cls()(
        plate_count=DEFAULT_PLATE_COUNT,
        plate_pitch_diameter_m=DEFAULT_PITCH_DIAMETER_M,
        plate_diameter_m=DEFAULT_PLATE_DIAMETER_M,
        collision_outer_diameter_m=DEFAULT_COLLISION_OUTER_DIAMETER_M,
        collision_top_offset_m=DEFAULT_COLLISION_TOP_OFFSET_M,
        collision_bottom_offset_m=DEFAULT_COLLISION_BOTTOM_OFFSET_M,
    )


def test_existing_twelve_point_math_matches_50cm_pitch_circle() -> None:
    basket = cooperative_trajectory.StaticRigidBodyTrajectory(np.zeros(3))
    points = cooperative_trajectory.basket_attachment_positions(
        basket.state(0.0), attachment_count=12, basket_radius_m=0.25
    )

    assert points.shape == (12, 3)
    radial = points[:, :2]
    np.testing.assert_allclose(np.linalg.norm(radial, axis=1), 0.25, atol=1e-12)
    angles = np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))
    np.testing.assert_allclose(np.diff(angles), 2.0 * math.pi / 12.0, atol=1e-12)


def test_primary_four_attachment_plates_are_quarter_turns_apart() -> None:
    basket = cooperative_trajectory.StaticRigidBodyTrajectory(np.zeros(3))
    points = cooperative_trajectory.basket_attachment_positions(
        basket.state(0.0), attachment_count=12, basket_radius_m=0.25
    )
    primary = points[[0, 3, 6, 9], :2]
    expected = np.array(
        [
            [0.25, 0.0],
            [0.0, 0.25],
            [-0.25, 0.0],
            [0.0, -0.25],
        ]
    )
    np.testing.assert_allclose(primary, expected, atol=1e-12)


def test_existing_yaw_kinematics_keep_plate_zero_on_plus_x_at_zero_yaw() -> None:
    basket = cooperative_trajectory.StaticRigidBodyTrajectory(np.array([2.0, 2.0, 1.5]))
    plate0 = cooperative_trajectory.AttachmentPointTrajectory(
        basket=basket,
        attachment_id=0,
        attachment_count=12,
        basket_radius_m=0.25,
        attachment_z_offset_m=0.0,
    )
    np.testing.assert_allclose(plate0.state(0.0).position, [2.25, 2.0, 1.5], atol=1e-12)


def test_ring_geometry_defaults_encode_agreed_physical_contract() -> None:
    geometry = _default_geometry()

    assert geometry.plate_count == 12
    assert geometry.plate_pitch_diameter_m == pytest.approx(0.50)
    assert geometry.plate_pitch_radius_m == pytest.approx(0.25)
    assert geometry.plate_diameter_m == pytest.approx(0.06)
    assert geometry.collision_outer_diameter_m == pytest.approx(0.56)
    assert geometry.collision_top_offset_m == pytest.approx(0.0)
    assert geometry.collision_bottom_offset_m == pytest.approx(-0.27)


def test_ring_geometry_plate_zero_is_plus_x_in_attachment_plate_plane() -> None:
    geometry = _default_geometry()
    np.testing.assert_allclose(
        geometry.plate_body_offset(0),
        np.array([0.25, 0.0, 0.0]),
        atol=1e-12,
    )



def test_plate_zero_preferred_approach_is_radially_inward() -> None:
    geometry = _default_geometry()
    np.testing.assert_allclose(
        geometry.plate_outward_direction_body(0),
        np.array([1.0, 0.0, 0.0]),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        geometry.plate_inward_approach_direction_body(0),
        np.array([-1.0, 0.0, 0.0]),
        atol=1e-12,
    )

def test_ring_geometry_plate_spacing_and_configurable_pitch_diameter() -> None:
    cls = _ring_geometry_cls()
    geometry = cls(
        plate_count=12,
        plate_pitch_diameter_m=0.60,
        plate_diameter_m=0.055,
        collision_outer_diameter_m=0.66,
        collision_top_offset_m=0.0,
        collision_bottom_offset_m=-0.25,
    )

    offsets = np.asarray([geometry.plate_body_offset(i) for i in range(12)])
    np.testing.assert_allclose(np.linalg.norm(offsets[:, :2], axis=1), 0.30, atol=1e-12)
    angles = np.unwrap(np.arctan2(offsets[:, 1], offsets[:, 0]))
    np.testing.assert_allclose(np.diff(angles), 2.0 * math.pi / 12.0, atol=1e-12)




def test_radially_tensioned_endpoint_matches_45deg_preview_geometry() -> None:
    geometry = _default_geometry()
    cable_length = 0.50
    angle = math.radians(45.0)
    horizontal = cable_length * math.sin(angle)
    vertical = cable_length * math.cos(angle)

    expected = {
        3: np.array([0.0, 0.25 + horizontal, vertical]),
        6: np.array([-0.25 - horizontal, 0.0, vertical]),
        9: np.array([0.0, -0.25 - horizontal, vertical]),
    }
    for plate_index, target in expected.items():
        np.testing.assert_allclose(
            geometry.radially_tensioned_endpoint_body(
                plate_index,
                cable_length_m=cable_length,
                angle_from_vertical_rad=angle,
            ),
            target,
            atol=1e-12,
        )

def test_ring_geometry_full_rotation_transforms_plate_position() -> None:
    geometry = _default_geometry()
    ring_position = np.array([1.0, 2.0, 3.0])

    # Rotate +x_R onto -z_W. This deliberately exercises a non-yaw 3-D rotation
    # even though current M1 keeps the ring level.
    rotation_world_from_ring = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )

    world = geometry.plate_position_world(
        ring_position=ring_position,
        rotation_world_from_ring=rotation_world_from_ring,
        plate_index=0,
    )
    np.testing.assert_allclose(world, [1.0, 2.0, 2.75], atol=1e-12)


def test_ring_collision_envelope_is_below_attachment_plane_and_separate_from_pitch() -> None:
    geometry = _default_geometry()
    np.testing.assert_allclose(geometry.collision_half_extents, [0.28, 0.28, 0.135], atol=1e-12)
    np.testing.assert_allclose(geometry.collision_center_offset, [0.0, 0.0, -0.135], atol=1e-12)

    # The collision footprint is deliberately independent of pitch-circle diameter.
    assert geometry.collision_outer_diameter_m != geometry.plate_pitch_diameter_m


def test_ring_geometry_rejects_invalid_dimensions() -> None:
    cls = _ring_geometry_cls()
    with pytest.raises(ValueError):
        cls(
            plate_count=12,
            plate_pitch_diameter_m=0.50,
            plate_diameter_m=0.06,
            collision_outer_diameter_m=0.56,
            collision_top_offset_m=-0.10,
            collision_bottom_offset_m=0.0,
        )
    with pytest.raises(ValueError):
        cls(
            plate_count=0,
            plate_pitch_diameter_m=0.50,
            plate_diameter_m=0.06,
            collision_outer_diameter_m=0.56,
            collision_top_offset_m=0.0,
            collision_bottom_offset_m=-0.27,
        )


def test_m2a_plate_frame_zero_is_identity_in_ring_frame() -> None:
    """M2A plate frame uses outward/tangent/normal axes.

    Plate 0 is on +x_R, so its local +x points +x_R, +y points +y_R,
    and +z follows the ring normal.
    """

    geometry = _default_geometry()
    rotation_ring_from_plate = geometry.plate_rotation_body(0)
    np.testing.assert_allclose(rotation_ring_from_plate, np.eye(3), atol=1e-12)


def test_m2a_plate_frame_cardinal_axes_follow_plate_index() -> None:
    geometry = _default_geometry()

    expected = {
        0: np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        3: np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        6: np.array(
            [
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        9: np.array(
            [
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
    }

    for plate_index, target in expected.items():
        rotation_ring_from_plate = geometry.plate_rotation_body(plate_index)
        np.testing.assert_allclose(rotation_ring_from_plate, target, atol=1e-12)
        np.testing.assert_allclose(
            rotation_ring_from_plate.T @ rotation_ring_from_plate,
            np.eye(3),
            atol=1e-12,
        )
        assert np.linalg.det(rotation_ring_from_plate) == pytest.approx(1.0)


def test_m2a_plate_rotation_world_composes_full_3d_ring_rotation() -> None:
    geometry = _default_geometry()

    rotation_world_from_ring = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )

    rotation_world_from_plate = geometry.plate_rotation_world(
        rotation_world_from_ring=rotation_world_from_ring,
        plate_index=3,
    )
    expected = rotation_world_from_ring @ geometry.plate_rotation_body(3)
    np.testing.assert_allclose(rotation_world_from_plate, expected, atol=1e-12)


def test_m2a_plate_rotation_world_rejects_non_rotation_input() -> None:
    geometry = _default_geometry()
    with pytest.raises(ValueError):
        geometry.plate_rotation_world(
            rotation_world_from_ring=np.diag([1.0, 1.0, 2.0]),
            plate_index=0,
        )
