"""sim_magnet: the RViz MAGNET toggles release only real tethers, and only once."""
import rclpy

from simulation_communication.sim_magnet import SimMagnet, magnet_action


def test_magnet_action_table():
    assert magnet_action('OFF', True, True) == 'release'
    assert magnet_action(' off ', True, True) == 'release'
    assert magnet_action('OFF', True, False) == 'none'
    assert magnet_action('ON', True, True) == 'none'
    assert magnet_action('ON', True, False) == 'cannot_reattach'
    assert magnet_action('OFF', False, False) == 'newcomer'
    assert magnet_action('ON', False, False) == 'newcomer'
    assert magnet_action('toggle', True, True) == 'ignore'


def test_node_topics_newcomer_has_no_release():
    rclpy.init(args=['--ros-args', '-p', 'num_drones:=4', '-p', 'num_tethers:=3'])
    try:
        node = SimMagnet()
        pubs = {t for t, _ in node.get_publisher_names_and_types_by_node('sim_magnet', '/')}
        subs = {t for t, _ in node.get_subscriber_names_and_types_by_node('sim_magnet', '/')}
        assert {f'/drone_{i}/magnet_release' for i in range(3)} <= pubs
        assert '/drone_3/magnet_release' not in pubs
        assert {f'/drone_{i}/magnet' for i in range(4)} <= subs
        assert not any('attach' in t and 'detachable' not in t for t in pubs | subs)
        node.destroy_node()
    finally:
        rclpy.shutdown()
