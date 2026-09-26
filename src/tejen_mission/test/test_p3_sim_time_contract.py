"""P3 timing-domain contracts for M2C simulation.

These tests are intentionally source-level / ROS-independent. P3's simulation
ROS nodes use Gazebo /clock consistently; only external host/process supervision
and computation profiling remain wall-time.
"""

from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
BASE_LAUNCH = REPO / "src/tejen_mission/launch/m2c_four_drone_ground.launch.py"
CONTROLLER_LAUNCH = REPO / "src/tejen_mission/launch/m2c_vehicle_controller.launch.py"
RUNNER = REPO / "tools/sim_test/run_m2c_ground.sh"
TRUTH = REPO / "src/tejen_mission/tejen_mission/m2a_gazebo_joint_truth_bridge.py"
FLEET = REPO / "src/tejen_mission/tejen_mission/m2_fleet_manager.py"
PLANNER = REPO / "src/tejen_mission/tejen_mission/online_join_planner.py"
CONTROLLER = REPO / "src/tejen_mpc/tejen_mpc/main.py"
CALLBACKS = REPO / "src/tejen_utility_objects/tejen_utility_objects/callback_manager.py"
RVIZ = REPO / "src/drone_visualisation/launch/view_frame.launch.py"
MOCAP = REPO / "src/simulation_communication/simulation_communication/motion_capture_emulator.py"
PENDULUM = REPO / "src/tejen_mission/tejen_mission/pendulum_state_publisher.py"
BETAFLIGHT = REPO / "src/simulation_communication/simulation_communication/tejen_betaflight_communication.py"


def test_p3_base_launch_bridges_clock_and_assigns_clock_domain_explicitly():
    text = BASE_LAUNCH.read_text(encoding="utf-8")
    assert text.count('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock') == 1
    assert 'name="m2c_clock_bridge"' in text

    # Physical simulation/control nodes must follow Gazebo /clock.
    for executable in (
        'tejen_motion_capture_emulator',
        'tejen_betaflight_communication',
        'pendulum_state_publisher',
        'online_join_planner',
        'm2c_fleet_manager',
    ):
        pos = text.index(f'executable="{executable}"')
        block = text[pos:pos + 2600]
        assert '"use_sim_time": True' in block, executable

    # The truth bridge is also a simulation ROS node and must follow /clock.
    pos = text.index('executable="m2a_gazebo_joint_truth_bridge"')
    block = text[pos:pos + 1800]
    assert '"use_sim_time": True' in block
    assert '"use_sim_time": False' not in text


def test_p3_controller_launch_uses_sim_time_without_changing_p1_cache_contract():
    text = CONTROLLER_LAUNCH.read_text(encoding="utf-8")
    assert '"use_sim_time": True' in text
    assert '"acados_cache_dir": str(cache_dir)' in text
    assert '"disarmed_external_reference_fast_path": True' in text


def test_p3_runner_waits_for_clock_before_pause_and_confirms_fleet_truth_after_unpause():
    text = RUNNER.read_text(encoding="utf-8")
    assert 'wait_for_sim_clock' in text
    clock_wait = text.index('wait_for_sim_clock')
    # Pick the invocation, not the function definition.
    clock_wait = text.index('wait_for_sim_clock ', clock_wait + 1)
    pause = text.index("--req 'pause: true'", clock_wait)
    assert clock_wait < pause

    release_loop = text.index('release_joint_until_detached "$i"')
    unpause = text.index("--req 'pause: false'", release_loop)
    fleet_confirm = text.index('wait_for_joint_detached_count 4', unpause)
    assert release_loop < unpause < fleet_confirm
    paused_block = text[release_loop:unpause]
    assert 'wait_for_joint_detached_count "$((i + 1))"' not in paused_block

    # Runner timeouts remain wall based even after ROS uses simulation time.
    assert 'started="$(date +%s)"' in text
    assert 'timeout "$timeout_s" ros2 topic echo' in text


def test_p3_rviz_launch_is_clock_configurable_and_m2c_requests_sim_time():
    rviz = RVIZ.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument' in rviz
    assert '"use_sim_time"' in rviz or "'use_sim_time'" in rviz
    assert 'LaunchConfiguration' in rviz
    assert 'use_sim_time:=true' in runner
    assert 'p3_clock_policy=all_sim_ros' in runner


def test_p3_existing_physical_derivative_nodes_use_node_clock():
    for path in (MOCAP, PENDULUM, BETAFLIGHT):
        text = path.read_text(encoding="utf-8")
        assert 'self.get_clock().now()' in text, path.name

    fleet = FLEET.read_text(encoding="utf-8")
    assert 'return self.get_clock().now().nanoseconds * 1e-9' in fleet


def test_p3_controller_uses_node_clock_for_ros_state_freshness_and_pose_watchdog():
    controller = CONTROLLER.read_text(encoding="utf-8")
    callbacks = CALLBACKS.read_text(encoding="utf-8")

    assert 'def _node_now_s(self)' in controller
    assert 'self.last_external_reference_time = self._node_now_s()' in controller
    assert 'age = self._node_now_s() - self.last_external_reference_time' in controller
    assert 'now = self._node_now_s() if now_s is None else float(now_s)' in controller

    # Pose and pendulum freshness are ROS simulation-time quantities.
    assert 'self.node.last_pose_update_time = (' in callbacks
    assert 'self.node.get_clock().now().nanoseconds * 1e-9' in callbacks
    assert '(self._node_now_s() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD' in controller

    # Pendulum freshness uses the same node clock.
    assert 'self.node.get_clock().now().nanoseconds * 1e-9' in callbacks
    assert 'last_pendulum_update_time' in callbacks


def test_p3_planner_has_explicit_physical_and_wall_clock_contracts():
    text = PLANNER.read_text(encoding="utf-8")
    assert 'def _physical_now_s(self)' in text
    assert 'return self.get_clock().now().nanoseconds * 1e-9' in text
    assert 'def _wall_now_s(self)' in text
    assert 'return time.monotonic()' in text

    # Mission / physical timing follows node clock.
    assert 'self.phase_entry_time = self._physical_now_s()' in text
    assert 'now = self._physical_now_s()' in text

    # ROS-side bootstrap truth and gate timing also follow the node clock.
    assert 'self.m2b_last_joint_truth_time_s = self._physical_now_s()' in text
    assert 'self.m2b_bootstrap_gate.release(now_s=self._physical_now_s())' in text


def test_p3_truth_bridge_is_timer_driven_and_sim_clocked_by_launch():
    text = TRUTH.read_text(encoding="utf-8")
    assert 'self.create_timer(self.republish_period_s, self._health)' in text


def _direct_time_calls(path: Path):
    import ast

    tree = ast.parse(path.read_text(encoding="utf-8"))
    calls = []

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.functions = []

        def visit_FunctionDef(self, node):
            self.functions.append(node.name)
            self.generic_visit(node)
            self.functions.pop()

        def visit_Call(self, node):
            fn = node.func
            if (
                isinstance(fn, ast.Attribute)
                and isinstance(fn.value, ast.Name)
                and fn.value.id == "time"
                and fn.attr in {"time", "monotonic", "perf_counter"}
            ):
                calls.append((self.functions[-1] if self.functions else "", fn.attr))
            self.generic_visit(node)

    Visitor().visit(tree)
    return calls


def test_p3_planner_has_no_unclassified_wall_clock_in_physical_mission_logic():
    # The only direct Python clocks left are the explicit computation wall helper,
    # CSV wall timestamp, and callback profiling. ROS mission/state timing uses
    # _physical_now_s().
    assert _direct_time_calls(PLANNER) == [
        ("_wall_now_s", "monotonic"),
        ("write_csv_log", "time"),
        ("timer_callback", "perf_counter"),
        ("timer_callback", "perf_counter"),
    ]


def test_p3_controller_direct_wall_clocks_are_only_diagnostics_watchdog_and_logging():
    calls = _direct_time_calls(CONTROLLER)
    assert calls == [
        ("_publish_c1d_status_if_due", "monotonic"),
        ("control_loop", "perf_counter"),
        ("control_loop", "perf_counter"),
        ("control_loop", "perf_counter"),
        ("control_loop", "perf_counter"),
        ("control_loop", "perf_counter"),
        ("control_loop", "perf_counter"),
        ("control_loop", "time"),  # human-facing CSV wall timestamp
        ("control_loop", "perf_counter"),
    ]
