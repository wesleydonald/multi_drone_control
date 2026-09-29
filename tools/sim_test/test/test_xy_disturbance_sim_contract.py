from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / "simulation_assets" / "tejen" / "world_drone_env_detach.sdf"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "c1f6_system_test.launch.py"
GATE = ROOT / "src" / "tejen_mission" / "tejen_mission" / "simulation_xy_disturbance_gate.py"
SETUP = ROOT / "src" / "tejen_mission" / "setup.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_sim_test.py"


def test_world_exposes_inert_apply_link_wrench_hook():
    text = WORLD.read_text()
    assert "apply-link-wrench-system" in text
    assert "ApplyLinkWrench" in text
    # No built-in nonzero force: ordinary simulation remains unchanged.
    assert "<persistent>" not in text


def test_c1f6_launch_plumbs_bias_mode_and_pickup_only_xy_force_window():
    text = LAUNCH.read_text()
    assert 'LaunchConfiguration("xy_bias_mode")' in text
    assert 'LaunchConfiguration("disturbance_force_x_n")' in text
    assert 'LaunchConfiguration("disturbance_force_y_n")' in text
    assert 'DeclareLaunchArgument("xy_bias_mode", default_value="legacy_integral")' in text
    assert 'DeclareLaunchArgument("disturbance_force_x_n", default_value="0.0")' in text
    assert 'DeclareLaunchArgument("disturbance_force_y_n", default_value="0.0")' in text
    assert '"xy_bias_mode": ParameterValue(xy_bias_mode, value_type=str)' in text
    assert 'executable="simulation_xy_disturbance_gate"' in text
    assert '"activation_phase": "SETTLE_ABOVE_PICKUP"' in text
    assert '"deactivation_phase": "LIFT_OBJECT"' in text
    assert '"activation_phase": "TRANSIT_TO_DROP_POINT"' not in text
    assert '"target_link": "x3::X3/base_link"' in text
    # The launch must not publish a persistent wrench immediately at startup.
    assert 'name="c1f6_apply_xy_disturbance"' not in text
    assert '"/world/quadcopter/wrench/persistent"' not in text


def test_disturbance_gate_targets_only_scoped_x3_base_link_during_pickup_window():
    text = GATE.read_text()
    assert 'target_link: str = "x3::X3/base_link"' in text
    assert '"activation_phase", "SETTLE_ABOVE_PICKUP"' in text
    assert '"deactivation_phase", "LIFT_OBJECT"' in text
    assert 'f"/world/{self.world_name}/wrench/persistent"' in text
    assert 'f"/world/{self.world_name}/wrench/clear"' in text
    assert '"gz.msgs.EntityWrench"' in text
    assert '"gz.msgs.Entity"' in text
    assert 'type: 3' in text  # gz.msgs.Entity LINK
    assert 'payload_model' not in text
    assert 'phase == self.activation_phase' in text
    assert 'phase == self.deactivation_phase' in text
    assert 'subprocess.run(' in text


def test_disturbance_gate_is_installed_as_ros_executable():
    text = SETUP.read_text()
    assert (
        "'simulation_xy_disturbance_gate = "
        "tejen_mission.simulation_xy_disturbance_gate:main'"
    ) in text


def test_sim_runner_exposes_ab_switch_without_changing_default():
    text = RUNNER.read_text()
    assert 'default="legacy_integral"' in text
    assert '"lateral_disturbance"' in text
    assert '"lateral_disturbance_shadow"' in text
    assert '"--disturbance-force-x-n"' in text
    assert '"--disturbance-force-y-n"' in text
    assert 'f"xy_bias_mode:={xy_bias_mode}"' in text
    assert 'f"disturbance_force_x_n:={float(disturbance_force_x_n)}"' in text
    assert 'f"disturbance_force_y_n:={float(disturbance_force_y_n)}"' in text


def test_c1f6_launch_exposes_automated_state_machine_landing_check():
    text = LAUNCH.read_text()
    assert 'DeclareLaunchArgument("supervisor_success_phase", default_value="ATTACH_READY")' in text
    assert 'DeclareLaunchArgument("supervisor_failure_phase", default_value="LANDED_DISARMED")' in text
    assert 'DeclareLaunchArgument("supervisor_require_handoff_ready", default_value="true")' in text
    assert 'DeclareLaunchArgument("supervisor_landing_request_phase", default_value="")' in text
    assert '"landing_request_phase": ParameterValue' in text
