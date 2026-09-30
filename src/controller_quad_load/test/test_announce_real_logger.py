"""The manager's status helper with a REAL rclpy logger: rclpy fixes the severity per
calling line, and one shared line killed the manager at the rig's first DISARM (2026-09-30)."""
import rclpy

from controller_quad_load.main import _announce


def test_announce_mixes_levels_without_raising():
    rclpy.init()
    try:
        node = rclpy.create_node('announce_test')
        for level in ('info', 'error', 'warn', 'info', 'error'):
            _announce(node, level, f'{level} line')
        node.destroy_node()
    finally:
        rclpy.shutdown()
