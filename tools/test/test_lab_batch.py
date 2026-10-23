"""
tools/remote/lab_batch.py: slot choice under the lab core budget, run-id reservation on the
laptop, the per-slot container command, and run_experiment's --run-dir / launch-arg checks.
No ssh: the lab-facing calls are exercised by the dry run and one real run (report).
"""
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(TOOLS, 'remote'))

import lab_batch as LB  # noqa: E402
import run_experiment as R  # noqa: E402
from experiment.config import ExperimentConfig  # noqa: E402

REPO = os.path.dirname(TOOLS)
CFG = os.path.join(REPO, 'configs', 'experiments', 'rig_twin_hover_fixed.yaml')


def _load(others=0.0, in_use=(), ncpu=24):
    return {'ncpu': ncpu, 'busy': others, 'ours': 0.0, 'others': others, 'slots_in_use': list(in_use)}


def test_idle_lab_gets_six_slots_of_four_cores():
    assert LB.pick_slots(_load(0.5), 6, 4, 10) == [1, 2, 3, 4, 5, 6]


def test_other_users_load_shrinks_the_batch():
    assert LB.pick_slots(_load(9.0), 6, 4, 10) == [1, 2, 3]
    assert LB.pick_slots(_load(17.0), 6, 4, 10) == [1]
    assert LB.pick_slots(_load(22.0), 6, 4, 10) == []


def test_slot_count_stops_at_six():
    assert LB.pick_slots(_load(0.0), 8, 3, 10) == [1, 2, 3, 4, 5, 6]


def test_running_slots_are_skipped_and_count_against_the_budget():
    assert LB.pick_slots(_load(0.0, in_use=[1, 2]), 6, 4, 10) == [3, 4, 5, 6]
    assert LB.pick_slots(_load(0.0, in_use=[1, 2, 3, 4, 5, 6]), 6, 4, 10) == []


def test_never_more_slots_than_configs():
    assert LB.pick_slots(_load(0.0), 4, 4, 1) == [1]


def test_cpu_busy_from_proc_stat():
    a = 'cpu  100 0 100 800 0 0 0 0 0 0'
    b = 'cpu  150 0 150 900 0 0 0 0 0 0'          # 100 busy of 200 jiffies
    assert LB._cpu_busy(a, b) == pytest.approx(0.5)


def test_dry_reservation_creates_nothing(tmp_path, monkeypatch):
    monkeypatch.setenv('MDC_RESULTS_ROOT', str(tmp_path))
    cfg = ExperimentConfig.from_yaml(CFG)
    jobs = LB.reserve([(CFG, cfg), (CFG, cfg)], '2026-10-01', dry=True)
    assert [j.rid for j in jobs] == ['R0001', 'R0002']
    assert not any(tmp_path.iterdir())


def test_reservation_takes_ids_past_existing_runs(tmp_path, monkeypatch):
    monkeypatch.setenv('MDC_RESULTS_ROOT', str(tmp_path))
    (tmp_path / '2026-09-30' / 'R0771_sim_gz_x').mkdir(parents=True)
    cfg = ExperimentConfig.from_yaml(CFG)
    jobs = LB.reserve([(CFG, cfg), (CFG, cfg)], '2026-10-01', dry=False)
    assert [j.rid for j in jobs] == ['R0772', 'R0773']
    for j in jobs:
        assert os.path.basename(j.local_dir) == f'{j.rid}_sim_gz_rig_twin_hover_fixed'
        assert os.path.exists(os.path.join(j.local_dir, 'RESERVED.txt'))


def test_slot_command_isolates_the_container():
    cfg = ExperimentConfig.from_yaml(CFG)
    job = LB.Job(CFG, cfg, 'R0900', '/x/results/2026-10-01/R0900_sim_gz_rig_twin_hover_fixed', '2026-10-01')
    cmd = job.command(3, 4, 10)
    s = ' '.join(cmd)
    assert '--tag slot3' in s and '--domain 43' in s and '--partition wesley_3' in s
    assert '--mount mdc_slots/slot3:/scratch/slot3' in s
    for env in ('MDC_ACADOS_ROOT=/scratch/slot3', 'MDC_LAUNCH_CWD=/scratch/slot3',
                'MDC_RESULTS_ROOT=/scratch/slot3/results'):
        assert env in cmd
    k = cmd.index('--run-dir')
    assert cmd[k + 1] == '/scratch/slot3/results/2026-10-01/R0900_sim_gz_rig_twin_hover_fixed'
    assert 'configs/experiments/rig_twin_hover_fixed.yaml' in cmd


def test_slot_builds_both_solvers_before_the_run():
    cfg = ExperimentConfig.from_yaml(CFG)
    job = LB.Job(CFG, cfg, 'R0900', '/x/results/2026-10-01/R0900_sim_gz_rig_twin_hover_fixed', '2026-10-01')
    cmd = job.command(2, 4, 10)
    k = cmd.index('bash')
    warm = cmd[k + 2]
    assert warm.startswith('cd /scratch/slot2 && ')          # the planner's code dir is cwd-relative
    assert 'prebuild_planner.py 4 --load-mass 0.86 --drone-mass 0.55' in warm
    assert '_solver_is_fresh() or c.generate_ocp_controller()' in warm
    assert warm.endswith('exec "$@"') and cmd[k + 4:k + 6] == ['python3', 'tools/run_experiment.py']


def test_the_warm_up_can_build_what_the_config_loads():
    """--warm-from-config (card 2026-10-23_scalability): every resize size, the ring inertia and the rod model of the
    config, built in the slot before the run; off, the warm-up is the fleet size with its masses, as before."""
    cfg = ExperimentConfig.from_yaml(CFG)
    job = LB.Job(CFG, cfg, 'R0900', '/x/results/2026-10-01/R0900_sim_gz_rig_twin_hover_fixed', '2026-10-01')
    assert not job.warm_from_config
    job.warm_from_config = True
    cmd = job.command(1, 12, 10)
    warm = cmd[cmd.index('bash') + 2]
    assert warm.startswith('cd /scratch/slot1 && python3 /home/wesley/multi_drone_control/tools/prebuild_planner.py '
                           '--from-config /home/wesley/multi_drone_control/configs/experiments/'
                           'rig_twin_hover_fixed.yaml && ')
    assert '_solver_is_fresh() or c.generate_ocp_controller()' in warm and '--cpus 12' in ' '.join(cmd)


def test_a_resizing_config_warms_every_size_it_resizes_through():
    """A slot's cache holds one solver per size: a config whose planner resizes (every detach-mode OCP flight) gets
    the --from-config warm-up without the flag, so nothing another ring's batch evicted compiles in the ready window."""
    assert LB.planner_sizes(CFG) == [4]
    s8 = os.path.join(REPO, 'configs', 'experiments', 's8_chain4_R50_L100_it1.yaml')
    assert LB.planner_sizes(s8) == list(range(2, 9))
    assert LB.planner_sizes(os.path.join(REPO, 'configs', 'experiments', 'b4_hover_L100_ema.yaml')) == [2, 3, 4]


def test_preallocated_run_dir_needs_a_run_id(tmp_path):
    d, rid = R.preallocated(str(tmp_path / 'R0901_sim_gz_a'))
    assert rid == 'R0901' and os.path.isdir(os.path.join(d, 'logs'))
    with pytest.raises(SystemExit):
        R.preallocated(str(tmp_path / 'not_a_run'))


def test_undeclared_launch_args_are_caught(tmp_path):
    cfg = ExperimentConfig.from_yaml(CFG)
    assert R.undeclared_launch_args(cfg) == []
    cfg.launch_args['no_such_arg'] = 1
    assert R.undeclared_launch_args(cfg) == ['sim_control_launch.py: no_such_arg']


def test_launch_cwd_follows_the_slot(monkeypatch):
    monkeypatch.delenv('MDC_LAUNCH_CWD', raising=False)
    assert R.launch_cwd() == R.REPO
    monkeypatch.setenv('MDC_LAUNCH_CWD', '/scratch/slot2')
    assert R.launch_cwd() == '/scratch/slot2'


def test_sync_is_refused_while_a_slot_runs_on_the_shared_tree(monkeypatch):
    monkeypatch.setattr(LB, 'lab_load', lambda: _load(0.5, in_use=[2]))
    with pytest.raises(SystemExit, match='shared tree'):
        LB.main([CFG])


def test_a_live_slot_is_allowed_beside_a_batch_that_neither_syncs_nor_builds(monkeypatch):
    monkeypatch.setattr(LB, 'lab_load', lambda: _load(0.5, in_use=[2]))
    assert LB.main([CFG, '--dry-run', '--no-sync', '--no-build']) == 0


def test_provenance_carries_the_ignored_code_the_sync_ships():
    code, other = LB.synced_ignored_files()
    # an ignored-and-untracked tools/ file the sync ships must be listed; tracked ones never are
    assert 'tools/remote/lab_batch.py' not in code          # tracked since a013220
    assert all(f.startswith(('tools/', 'configs/', 'docs/', 'src/', 'simulation_assets/')) for f in code)
    assert not any(f.startswith(LB.SYNC_EXCLUDED) or '__pycache__' in f for f in code + other)


def test_a_hung_run_is_stopped_at_the_wall_cap(tmp_path, monkeypatch):
    """R1347: a runner that never ends holds its slot for hours; the cap stops this batch's container."""
    import queue
    import subprocess
    import threading

    class _P:
        def __init__(self, *a, **k):
            self.waits = 0

        def wait(self, timeout=None):
            self.waits += 1
            if self.waits == 1:
                raise subprocess.TimeoutExpired('run', timeout)
            return 137

    stopped = []
    monkeypatch.setattr(LB.subprocess, 'Popen', _P)
    monkeypatch.setattr(LB, 'lab', lambda cmd, **k: stopped.append(cmd) or '')
    monkeypatch.setattr(LB, 'fetch', lambda job, slot: False)

    class _Job:
        rid, rc, note, wall_s, passed, slot = 'R9999', None, '', None, None, None
        cfg = type('C', (), {'name': 'x'})()
        local_dir = str(tmp_path)

        def command(self, slot, cpus, gz_nice):
            return ['true']
    q = queue.Queue()
    job = _Job()
    q.put(job)
    LB.worker(3, q, 4, 10, {}, threading.Lock(), wall_cap_s=0.01)
    assert job.rc == 137 and 'WALL CAP' in job.note
    assert stopped and 'docker stop' in stopped[0] and 'wesley_slot3' in stopped[0]
