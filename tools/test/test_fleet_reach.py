"""tools/fleet_reach.py (critic F2): a terminal reaches the fleet when the fleet manager's latched
/fleet/manager_status answers; the fleet CLI refuses an unknown command before touching ROS."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import fleet_reach  # noqa: E402

rclpy = pytest.importorskip('rclpy')


def test_unknown_command_is_refused_without_ros(capsys):
    assert fleet_reach.main(['--send', 'arn']) == 2
    assert 'unknown command' in capsys.readouterr().err


def test_probe_answers_on_the_latched_status_and_not_without_it(monkeypatch):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(domain_id=96)
    try:
        asker = rclpy.create_node('asker')
        assert fleet_reach.probe(asker, 0.5) is None
        manager = rclpy.create_node('fake_manager')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        manager.create_publisher(String, '/fleet/manager_status', qos).publish(String(data='CentralController ready'))
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(manager)
        import threading
        th = threading.Thread(target=lambda: [ex.spin_once(timeout_sec=0.05) for _ in range(60)], daemon=True)
        th.start()
        assert fleet_reach.probe(asker, 2.0) == 'CentralController ready'
        th.join()
        manager.destroy_node()
        asker.destroy_node()
    finally:
        rclpy.shutdown()
