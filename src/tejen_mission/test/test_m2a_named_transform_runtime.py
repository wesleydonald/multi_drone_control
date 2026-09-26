import numpy as np
import pytest

from tejen_mission.m2a_attachment_runtime import (
    NamedTransform,
    resolve_named_transform_world,
)


def q_identity():
    return np.array([0.0, 0.0, 0.0, 1.0])


def test_resolve_named_transform_world_composes_pose_publisher_chain():
    transforms = [
        NamedTransform('quadcopter', 'm2a_carrier', [0.55, 0.0, 0.805], q_identity()),
        NamedTransform('m2a_carrier', 'carrier_link', [0.0, 0.0, 0.0], q_identity()),
        NamedTransform('carrier_link', 'tether_rod', [0.0, 0.0, 0.0], q_identity()),
        NamedTransform('tether_rod', 'magnet_tip_link', [0.0, 0.0, -0.5], q_identity()),
    ]
    p_c, R_c, root_c = resolve_named_transform_world(transforms, 'carrier_link')
    p_m, R_m, root_m = resolve_named_transform_world(transforms, 'magnet_tip_link')
    np.testing.assert_allclose(p_c, [0.55, 0.0, 0.805])
    np.testing.assert_allclose(p_m, [0.55, 0.0, 0.305])
    np.testing.assert_allclose(R_c, np.eye(3))
    np.testing.assert_allclose(R_m, np.eye(3))
    assert root_c == root_m == 'quadcopter'


def test_resolver_matches_namespaced_leaf_names():
    transforms = [
        NamedTransform('world', 'm2a_carrier', [1.0, 2.0, 3.0], q_identity()),
        NamedTransform('m2a_carrier', 'm2a_carrier::carrier_link', [0.0, 0.0, 0.0], q_identity()),
    ]
    p, _, _ = resolve_named_transform_world(transforms, 'carrier_link')
    np.testing.assert_allclose(p, [1.0, 2.0, 3.0])


def test_resolver_rejects_ambiguous_leaf_names():
    transforms = [
        NamedTransform('world', 'a::magnet_tip_link', [0, 0, 0], q_identity()),
        NamedTransform('world', 'b::magnet_tip_link', [0, 0, 0], q_identity()),
    ]
    with pytest.raises(ValueError, match='ambiguous'):
        resolve_named_transform_world(transforms, 'magnet_tip_link')


def test_resolver_rejects_cycles():
    transforms = [
        NamedTransform('b', 'a', [0, 0, 0], q_identity()),
        NamedTransform('a', 'b', [0, 0, 0], q_identity()),
    ]
    with pytest.raises(ValueError, match='cycle'):
        resolve_named_transform_world(transforms, 'a')
