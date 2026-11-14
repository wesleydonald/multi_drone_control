"""VIS2 backend source boundaries; scoped C++ build checks the ROS API."""
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / 'src/transfer_backend_node.cpp'


def markers():
    text = SOURCE.read_text()
    return text.split('void publishMarkers() noexcept {', 1)[1].split('void writeReplanCsvRow', 1)[0]


def test_unused_visualization_returns_before_snapshot_or_message_construction():
    body = markers()
    guard = body.index('marker_pub_->get_subscription_count() == 0U')
    intra = body.index('marker_pub_->get_intra_process_subscription_count() == 0U')
    returned = body.index('return;', max(guard, intra))
    assert guard < intra < returned < body.index('MarkerArray array;') < body.index('world_->snapshot(now_s)')


def test_wall_vertex_cloud_removed_but_required_markers_retained():
    body = markers()
    assert 'door_cspace_' not in body
    assert 'if (!static_obstacle_vertices_.empty())' not in body
    for name in ('physical_obstacle', 'cspace_', 'ego_physical_body', 'ego_physical_cable',
                 'ego_physical_magnet', 'ego_physical_payload', 'ego_tracking_vertices_',
                 'cooperative_shared_preview', 'cpp_reference', 'planning_goal', 'c1f3_status'):
        assert '"'+name in body
    assert 'Marker::DELETEALL' in body
    assert 'marker_pub_->publish(array);' in body
    text = SOURCE.read_text()
    assert 'marker_qos.reliable().transient_local();' in text
    assert 'declare_parameter<double>("marker_rate_hz", 5.0)' in text
    # Physical static collision inputs remain installed outside the display callback.
    assert 'static_obstacle_vertices_' in text.split('void publishMarkers()', 1)[0]
