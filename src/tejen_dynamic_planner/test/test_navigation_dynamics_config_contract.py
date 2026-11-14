"""ROS adapter wiring contracts; native limit propagation has its own tests."""
from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1]/'src/transfer_backend_node.cpp').read_text()


def test_backend_xy_dynamics_defaults_and_validation():
    assert '"planner_max_acceleration_xy_mps2", 1.0' in SOURCE
    assert '"planner_max_jerk_xy_mps3", 4.0' in SOURCE
    for member in ('planner_max_acceleration_xy_mps2_', 'planner_max_jerk_xy_mps3_'):
        assert f'!std::isfinite({member})' in SOURCE
        assert f'!({member} > 0.0)' in SOURCE


def test_shared_configuration_paths_preserve_z_and_backup_defaults():
    feature = SOURCE.split('RecedingHorizonConfig withPlannerFeatures(', 1)[1].split(
        'dynamic_planner::TrajectorySafetyChecker plannerSafetyChecker()', 1)[0]
    for axis in 'xy':
        assert f'config.a_max.{axis}() = planner_max_acceleration_xy_mps2_;' in feature
    assert 'config.a_max.z()' not in feature
    assert 'config.rendezvous_backup_brake_accel_limit' not in feature
    local = SOURCE.split('LocalPlannerConfig localConfig(', 1)[1].split('void validateParameters()', 1)[0]
    for axis in 'xy':
        assert f'config.refinement.j_max.{axis}() = planner_max_jerk_xy_mps3_;' in local
    assert 'config.refinement.j_max.z()' not in local
    assert SOURCE.count('withPlannerFeatures(recedingConfig(') == 6
    # Existing native constructor forwards shared acceleration to both stages.
    native = (Path(__file__).resolve().parents[1]/'standalone/src/receding_horizon_planner.cpp').read_text()
    assert 'local_config.octopus.a_max = config_.a_max;' in native
    assert 'local_config.refinement.a_max = config_.a_max;' in native
