from pathlib import Path
import sys
import types

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
DRONE = ROOT / "src/tejen_mission"
CONFIG = DRONE / "config/irl_commissioning.yaml"
MAGNET_SOURCE = DRONE / "tejen_mission/magnet_attachment_manager_irl.py"
PLANNER_SOURCE = DRONE / "tejen_mission/online_join_planner.py"


def _import_manager_module_without_ros():
    """Import the IRL manager in offline test environments without ROS 2."""
    try:
        import rclpy  # noqa: F401
    except ModuleNotFoundError:
        rclpy = types.ModuleType("rclpy")
        rclpy_node = types.ModuleType("rclpy.node")

        class Node:
            pass

        rclpy_node.Node = Node
        rclpy.node = rclpy_node
        sys.modules.setdefault("rclpy", rclpy)
        sys.modules.setdefault("rclpy.node", rclpy_node)

        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

        class PoseArray:
            pass

        class PoseStamped:
            pass

        geometry_msgs_msg.PoseArray = PoseArray
        geometry_msgs_msg.PoseStamped = PoseStamped
        sys.modules.setdefault("geometry_msgs", geometry_msgs)
        sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")

        class Bool:
            def __init__(self):
                self.data = False

        class String:
            def __init__(self):
                self.data = ""

        std_msgs_msg.Bool = Bool
        std_msgs_msg.String = String
        sys.modules.setdefault("std_msgs", std_msgs)
        sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

        interfaces = types.ModuleType("interfaces")
        interfaces_msg = types.ModuleType("interfaces.msg")

        class ELRSCommand:
            pass

        interfaces_msg.ELRSCommand = ELRSCommand
        sys.modules.setdefault("interfaces", interfaces)
        sys.modules.setdefault("interfaces.msg", interfaces_msg)

    import tejen_mission.magnet_attachment_manager_irl as module

    return module


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))

    warning = warn


def _attached_manager(module, *, now=10.0):
    manager = module.MagnetAttachmentManagerIRL.__new__(
        module.MagnetAttachmentManagerIRL
    )
    manager.magnet_on = True
    manager.assumed_attached = True
    manager.attachment_loss_latched = False
    manager.attach_condition_start = None
    manager.detach_condition_start = None
    manager.attach_radius_m = 0.12
    manager.attach_speed_threshold_mps = 0.25
    manager.attach_dwell_s = 0.15
    manager.detach_dwell_s = 0.20
    manager.pose_timeout_s = 0.25
    manager.tip_contact_position = np.zeros(3)
    manager.pickup_contact_position = np.zeros(3)
    manager.tip_contact_velocity = np.zeros(3)
    manager.pickup_contact_velocity = np.zeros(3)
    manager.last_tip_time = now
    manager.last_pickup_time = now
    manager.attached_pub = _Publisher()
    manager.state_pub = _Publisher()
    manager.publish_elrs = lambda force=False: None
    manager.get_logger = lambda: _Logger()
    return manager


def test_irl_config_uses_one_detach_dwell_without_extra_thresholds():
    data = yaml.safe_load(CONFIG.read_text())
    magnet = data["magnet_attachment_manager_irl"]["ros__parameters"]
    assert magnet["attach_radius_m"] == 0.12
    assert magnet["attach_dwell_s"] == 0.15
    assert magnet["detach_dwell_s"] == 0.20
    assert "detach_radius_m" not in magnet
    assert "detach_speed_threshold_mps" not in magnet


def test_brief_geometric_separation_does_not_detach(monkeypatch):
    module = _import_manager_module_without_ros()
    now = 10.0
    manager = _attached_manager(module, now=now)
    manager.pickup_contact_position = np.array([0.13, 0.0, 0.0])

    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert manager.assumed_attached

    now = 10.19
    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert manager.assumed_attached


def test_sustained_geometric_separation_detaches_and_stays_unlatched(monkeypatch):
    module = _import_manager_module_without_ros()
    now = 20.0
    manager = _attached_manager(module, now=now)
    manager.pickup_contact_position = np.array([0.13, 0.0, 0.0])

    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    now = 20.21
    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert not manager.assumed_attached
    assert manager.attachment_loss_latched

    # Even if the two tracked contact points later become close while the magnet
    # remains ON, a new pickup attempt is not armed until the normal OFF/ON cycle.
    manager.pickup_contact_position = np.array([0.01, 0.0, 0.0])
    now = 20.50
    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert not manager.assumed_attached


def test_relative_speed_alone_does_not_detach(monkeypatch):
    module = _import_manager_module_without_ros()
    now = 30.0
    manager = _attached_manager(module, now=now)
    manager.pickup_contact_position = np.array([0.01, 0.0, 0.0])
    manager.tip_contact_velocity = np.array([2.0, 0.0, 0.0])

    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    now = 30.30
    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert manager.assumed_attached


def test_sustained_stale_pose_detaches_after_loss_dwell(monkeypatch):
    module = _import_manager_module_without_ros()
    now = 40.30
    manager = _attached_manager(module, now=40.0)

    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert manager.assumed_attached
    now = 40.51
    monkeypatch.setattr(module.time, "time", lambda: now)
    manager.timer_cb()
    assert not manager.assumed_attached



def test_magnet_off_rearms_attachment_after_declared_loss():
    module = _import_manager_module_without_ros()
    manager = _attached_manager(module, now=50.0)
    manager.assumed_attached = False
    manager.attachment_loss_latched = True
    manager.detach_condition_start = 49.8

    command = module.String()
    command.data = "OFF"
    manager.command_cb(command)

    assert not manager.magnet_on
    assert not manager.assumed_attached
    assert not manager.attachment_loss_latched
    assert manager.attach_condition_start is None
    assert manager.detach_condition_start is None

def test_planner_retries_existing_pickup_sequence_on_lift_attachment_loss():
    source = PLANNER_SOURCE.read_text()
    lift_start = source.index("if self.phase == MissionPhase.LIFT_OBJECT:")
    prepare_start = source.index("if self.c1f2_cpp_authority_enabled:", lift_start)
    prefix = source[lift_start:prepare_start]

    assert "if not self.object_attached:" in prefix
    assert "MissionPhase.APPROACH_ABOVE_PICKUP" in prefix
    assert "attachment lost during loaded lift" in prefix.lower()
    assert "reset_c1f2_for_pickup_retry" in prefix


def test_retry_cleanup_revokes_authority_and_clears_stale_prepare_state():
    source = PLANNER_SOURCE.read_text()
    start = source.index("def reset_c1f2_for_pickup_retry")
    end = source.index("\n    def ", start + 8)
    helper = source[start:end]

    required = (
        "c1f2_publish_authority(False)",
        "self.c1f2_prepare_active = False",
        "self.c1f2_prepare_settled = False",
        "self.c1f2_handoff_ready = False",
        "self.c1f2_authority_grant_pending = False",
        "self.c1f2_authority_acknowledged = False",
        "self.c1f2_cached_cpp_reference = None",
        "self.c1f1_shadow_latched_body_target = None",
    )
    for text in required:
        assert text in helper
