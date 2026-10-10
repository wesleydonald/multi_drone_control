"""Focused ground-ring geometry, authority and release-order contract."""

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from types import SimpleNamespace
from visualization_msgs.msg import Marker

from tejen_mission.fake_cooperative_transport_world import FakeCooperativeTransportWorld


def test_visual_labels_cannot_move_published_plate_or_obstacle_geometry():
    world = SimpleNamespace(
        append_ring_markers=lambda markers, payload_state, stamp: None,
        make_marker_common=lambda marker_id, namespace, kind, stamp: Marker(),
        get_float_param=lambda name: 0.06,
        ring_attachment_plate_index=0,
        attachment_trail=[],
    )
    plate_pose = PoseStamped()
    plate_pose.pose.position.z = 0.05
    obstacle = SimpleNamespace(
        state=SimpleNamespace(x=0.0, y=0.0, z=0.4, yaw=0.0),
        radius=0.10,
        safety_radius=0.15,
    )
    world.make_pose = FakeCooperativeTransportWorld.make_pose.__get__(world)
    markers = FakeCooperativeTransportWorld.make_markers(
        world, None, None, plate_pose, [obstacle], Time()
    ).markers
    assert np.isclose(plate_pose.pose.position.z, 0.05)
    assert np.isclose(markers[0].pose.position.z, 0.05)
    assert np.isclose(markers[1].pose.position.z, 0.17)
    assert np.isclose(markers[2].pose.position.z, 0.4)
    assert np.isclose(markers[3].pose.position.z, 0.4)
    assert np.isclose(markers[4].pose.position.z, 0.63)
