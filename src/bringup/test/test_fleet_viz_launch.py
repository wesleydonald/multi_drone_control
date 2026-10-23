"""fleet_viz's swung-magnet topics in the io launches (evaluated offline, nothing started): the rig draws QUAD4's
coil from its radio latch's topic (the coil gate's private one in the combined M1), the twin from Tejen's magnet
command, and the puck from the mocap body or his twin publisher."""
import pytest
pytest.importorskip('launch_ros')

from test_real_mode_launches import _by_name, evaluate  # noqa: E402


@pytest.fixture(autouse=True)
def _installed():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('bringup')
        get_package_share_directory('drone_visualisation')
    except Exception:
        pytest.skip('source install/setup.bash')


def _viz(launch_file, **args):
    return _by_name(evaluate(launch_file, **args))['/fleet_viz']['params']


def test_rig_fleet_viz_reads_the_tip_the_puck_and_quad4s_latch():
    p = _viz('real_io_launch.py', num_drones=4, attach=True)
    assert (p['tip_topic'], p['puck_topic'], p['tether_drone']) == (
        '/magnet_tip_pose', '/pickup/motion_capture_state', 3)
    assert (p['coil_topic'], p['attached_topic']) == ('/drone_3/magnet', '/magnet/object_attached')
    assert (p['marker_hz'], p['summary_hz'], p['viz_hz']) == (5.0, 5.0, 10.0)


def test_combined_rig_fleet_viz_reads_the_coil_gates_topic():
    p = _viz('real_io_launch.py', num_drones=4, attach=True, magnet_topic_overrides='3:/drone_3/magnet_coil')
    assert p['coil_topic'] == '/drone_3/magnet_coil'


def test_twin_fleet_viz_reads_his_puck_and_magnet_command():
    p = _viz('sim_io_launch.py', num_drones=3, attach=True, rviz=False)
    assert (p['num_drones'], p['tip_topic'], p['puck_topic'], p['coil_topic']) == (
        4, '/magnet_tip_pose', '/tejen/payload_world_state', '/tejen/magnet/command')
