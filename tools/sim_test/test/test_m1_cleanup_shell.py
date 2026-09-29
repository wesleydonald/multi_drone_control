from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CLEANUP = ROOT / "tools" / "sim_test" / "cleanup_m1_stale.sh"
WRAPPER = ROOT / "tools" / "sim_test" / "run_m1_mpc_disturbance.sh"


def test_m1_cleanup_refuses_to_kill_live_supervised_run_before_any_pkill():
    text = CLEANUP.read_text()
    guard = text.index('if [[ -n "$LIVE_PRIMARY" ]]')
    refusal = text.index('Refusing automatic cleanup')
    first_pkill = text.index('pkill -"$sig"')
    assert guard < refusal < first_pkill
    assert 'run_sim_test.py run' in text
    assert 'c1f6_system_test\\.launch\\.py' in text


def test_m1_cleanup_finalizes_or_preserves_stale_capture_marker():
    text = CLEANUP.read_text()
    assert 'experiment_capture.py" stop' in text
    assert '.active_run.stale_${STAMP}.json' in text
    assert 'mv "$ACTIVE_CAPTURE" "$BACKUP"' in text
    assert 'rm -f "$ACTIVE_CAPTURE"' not in text


def test_m1_cleanup_targets_m1_children_without_touching_security_or_packages():
    text = CLEANUP.read_text()
    for token in (
        'world_drone_env_detach.sdf',
        'online_join_planner',
        'simulation_xy_disturbance_gate',
        'dynamic_planner_transfer_backend',
        'tejen_mpc/lib/tejen_mpc/main',
    ):
        assert token in text
    assert 'sudo ' not in text
    assert '/etc/' not in text
    assert 'apt ' not in text
    assert 'gnome-keyring' not in text


def test_disturbance_wrapper_runs_cleanup_before_case_runner():
    text = WRAPPER.read_text()
    cleanup = text.index('cleanup_m1_stale.sh')
    runner = text.index('run_m1_case.sh')
    assert cleanup < runner


def test_m1_cleanup_sources_ros_before_enabling_nounset():
    text = CLEANUP.read_text()
    assert 'set -euo pipefail' not in text
    source_ros = text.index('source /opt/ros/humble/setup.bash')
    disable_nounset = text.rfind('set +u', 0, source_ros)
    enable_nounset = text.index('set -u', source_ros)
    assert disable_nounset < source_ros < enable_nounset
    workspace_source = text.index('source "$REPO/install/setup.bash"')
    assert source_ros < workspace_source < enable_nounset
