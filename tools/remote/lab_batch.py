#!/usr/bin/env python3
"""tools/remote/lab_batch.py -- run a batch of Gazebo experiment configs on the lab PC, in parallel.

    tools/remote/lab_batch.py configs/experiments/a.yaml configs/experiments/b.yaml
    tools/remote/lab_batch.py --dry-run configs/experiments/rig_twin_*.yaml
    tools/remote/lab_batch.py --slots 2 --no-build cfg.yaml

Runs on the laptop. In order:
  1. validates every config locally (run_experiment.py --check: world, masses, launch args);
  2. reads the lab load and picks the slot count: at most 6 (--slots), at most 24 cores in all,
     fewer when other users' load is over 1 core or wesley_slot<n> containers are already up;
  3. reserves the run ids HERE, by creating results/<date>/R####_sim_gz_<name>/ (with a
     RESERVED.txt) before anything runs: next_run_id is a directory scan, so ids allocated
     in parallel on the lab would collide;
  4. syncs the tree (lab_sync.sh) and builds it (lab_build.sh --ws) unless told not to; both
     are refused while any wesley_slot<n> is up, since every slot runs on that one tree;
  5. runs each config in its own container, wesley_slot<n>: ROS_DOMAIN_ID 40+n, GZ_PARTITION
     wesley_<n>, drones:~/mdc_slots/slot<n> mounted at /scratch/slot<n> and used for the
     generated solvers (MDC_ACADOS_ROOT, and the planner's relative code dir through
     MDC_LAUNCH_CWD) and the staged results (MDC_RESULTS_ROOT); the planner and tracker
     solvers are built there before the run starts (Job.warm_script);
  6. rsyncs each result back into the reserved directory, points its manifest at the laptop
     copy, and prints a summary. Registry rows stay a laptop job.

The laptop's git SHA, branch and full diff (tracked, untracked, and the git-ignored code the
sync ships), taken after the sync, go into every manifest: the lab tree is an rsync copy without
.git. Only this batch's own containers are ever removed (Ctrl-C removes the ones it started).
Exit status is non-zero if any run failed.
"""
import argparse
import datetime
import json
import os
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'tools'))

from experiment.config import ExperimentConfig    # noqa: E402
from run_dir import git_state, next_run_id         # noqa: E402

LAB = os.environ.get('LAB_HOST', 'drones@drones')
SLOTS_DIR = 'mdc_slots'              # under the lab user's home
CTR_REPO = '/home/wesley/multi_drone_control'
MAX_SLOTS = 6                        # hard ceiling; the default below is what stays in real time
DEFAULT_SLOTS = 4                    # 6 x 4 cores slowed Gazebo to RTF 0.25 and voided R0801/R0802/R0805
CORE_BUDGET = 24                     # all cores while nobody else uses the PC (Wesley, 30 Sep)
OTHERS_FREE = 1                      # other users' load above this comes off the budget
DOMAIN_BASE = 40
MAX_UNTRACKED_BYTES = 1 << 20


def lab(cmd, check=True, timeout=60):
    r = subprocess.run(['ssh', '-o', 'BatchMode=yes', LAB, cmd], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=timeout)
    out = r.stdout.decode(errors='replace')
    if check and r.returncode != 0:
        raise SystemExit(f'lab command failed ({r.returncode}): {cmd}\n{out}')
    return out


# ── lab load and slots ───────────────────────────────────────────────────────

def _cpu_busy(a, b):
    """Busy cores between two /proc/stat 'cpu' lines, times the core count."""
    x = [int(v) for v in a.split()[1:]]
    y = [int(v) for v in b.split()[1:]]
    d = [q - p for p, q in zip(x, y)]
    total = sum(d)
    idle = d[3] + (d[4] if len(d) > 4 else 0)
    return (total - idle) / total if total > 0 else 0.0


def lab_load():
    """{ncpu, busy, ours, others, slots_in_use}: cores in use over 2 s, split into this user's
    wesley_* containers and everyone else's."""
    out = lab("nproc; head -1 /proc/stat; sleep 2; head -1 /proc/stat; "
              "docker stats --no-stream --format '{{.Name}} {{.CPUPerc}}' 2>/dev/null; "
              "echo ---; docker ps --format '{{.Names}}'")
    lines = out.splitlines()
    ncpu = int(lines[0])
    busy = _cpu_busy(lines[1], lines[2]) * ncpu
    stats, names = lines[3:], []
    if '---' in stats:
        k = stats.index('---')
        stats, names = stats[:k], stats[k + 1:]
    ours = 0.0
    for ln in stats:
        parts = ln.split()
        if len(parts) == 2 and parts[0].startswith('wesley_'):
            try:
                ours += float(parts[1].rstrip('%')) / 100.0
            except ValueError:
                pass
    in_use = sorted(int(n[len('wesley_slot'):]) for n in names
                    if n.startswith('wesley_slot') and n[len('wesley_slot'):].isdigit())
    return {'ncpu': ncpu, 'busy': busy, 'ours': ours, 'others': max(0.0, busy - ours),
            'slots_in_use': in_use}


def pick_slots(load, want, cpus, n_configs):
    """Slot numbers to use: the core budget minus other users' load above OTHERS_FREE and minus
    this user's slots already running, shared out at `cpus` each."""
    others = load['others'] if load['others'] > OTHERS_FREE else 0.0     # background noise is not a user
    free_cores = load['ncpu'] - others - cpus * len(load['slots_in_use'])
    free_cores = min(free_cores, CORE_BUDGET - cpus * len(load['slots_in_use']))
    k = max(0, min(want, n_configs, MAX_SLOTS, int(free_cores // cpus)))
    free = [n for n in range(1, MAX_SLOTS + 1) if n not in load['slots_in_use']]
    return free[:k]


# ── provenance ───────────────────────────────────────────────────────────────

def _git(*args):
    return subprocess.run(['git', '-C', REPO, *args], stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL).stdout.decode(errors='replace')


# lab_sync.sh's excludes: what the lab never receives
SYNC_EXCLUDED = ('build/', 'install/', 'log/', 'logs/', 'results/', 'results_archive/')
# git-ignored paths the sync ships that hold code the runs execute (tools/ is ignored whole)
IGNORED_CODE_DIRS = ('tools/', 'configs/', 'src/', 'simulation_assets/')
CODE_SUFFIXES = ('.py', '.sh', '.yaml', '.yml', '.sdf', '.xml', '.urdf', '.json', '.csv')


def synced_ignored_files():
    """git-ignored files lab_sync.sh ships anyway: (code files, everything else)."""
    out = _git('ls-files', '--others', '--ignored', '--exclude-standard', '--', '.',
               *[f':!{d.rstrip("/")}' for d in SYNC_EXCLUDED])
    code, other = [], []
    for f in out.splitlines():
        if '__pycache__' in f or 'c_generated_code' in f or f.endswith('.pyc'):
            continue
        (code if f.startswith(IGNORED_CODE_DIRS) and f.endswith(CODE_SUFFIXES) else other).append(f)
    return code, other


def laptop_git_state():
    """git_state() plus the diff itself: tracked changes against HEAD, then every untracked
    file under 1 MB as a new-file diff, the git-ignored code the sync ships included. The
    other ignored files it ships (notes, caches) enter as one hash."""
    import hashlib
    st = git_state()
    diff = _git('diff', 'HEAD')
    skipped = []
    code, other = synced_ignored_files()
    for f in _git('ls-files', '--others', '--exclude-standard').splitlines() + code:
        p = os.path.join(REPO, f)
        if not os.path.isfile(p) or os.path.getsize(p) > MAX_UNTRACKED_BYTES:
            skipped.append(f)
            continue
        diff += _git('diff', '--no-index', '--', '/dev/null', f)
    h = hashlib.sha256()
    for f in sorted(other):
        try:
            with open(os.path.join(REPO, f), 'rb') as fh:
                h.update(f.encode() + b'\0' + hashlib.sha256(fh.read()).digest())
        except OSError:
            pass
    st.update(diff=diff, git_state_source='laptop (tools/remote/lab_batch.py)',
              untracked_skipped=skipped, ignored_code_files=len(code),
              ignored_other_sha256=h.hexdigest(), laptop_host=os.uname().nodename)
    return st


def slots_up():
    """wesley_slot<n> containers running on the lab now."""
    return sorted(n for n in lab("docker ps --format '{{.Names}}'").split()
                  if n.startswith('wesley_slot'))


# ── one run ──────────────────────────────────────────────────────────────────

class Job:
    def __init__(self, cfg_path, cfg, rid, local_dir, date):
        self.cfg_path, self.cfg, self.rid, self.local_dir, self.date = cfg_path, cfg, rid, local_dir, date
        self.name = os.path.basename(local_dir)
        self.slot = None
        self.rc = None
        self.wall_s = None
        self.passed = None
        self.note = ''

    def ctr_dir(self, slot):
        return f'/scratch/slot{slot}/results/{self.date}/{self.name}'

    def host_dir(self, slot):
        return f'{SLOTS_DIR}/slot{slot}/results/{self.date}/{self.name}'

    def warm_script(self, scratch):
        """Shell run in the slot before the timed run: builds (or loads) the planner OCP for
        this config's fleet size and masses and the tracker OCP in the slot's solver dirs, so
        nothing compiles during a flight (R0780, R0785 were void for that)."""
        la = self.cfg.launch_args
        n = int(la.get('num_drones', self.cfg.num_drones))
        masses = ''.join(f' --{k.replace("_", "-")} {float(la[k])}' for k in ('load_mass', 'drone_mass') if k in la)
        tracker = ('from controller_quad_load import controller_mpc as c\n'
                   'c._solver_is_fresh() or c.generate_ocp_controller()')
        planner = ('' if str(la.get('reference', 'planner')) == 'free_hover'
                   else f'python3 {CTR_REPO}/tools/prebuild_planner.py {n}{masses} && ')
        return (f'cd {scratch} && {planner}'
                f'python3 -c {shlex.quote(tracker)} && cd {CTR_REPO} && exec "$@"')

    def command(self, slot, cpus, gz_nice):
        rel = os.path.relpath(os.path.abspath(self.cfg_path), REPO)
        scratch = f'/scratch/slot{slot}'
        return [os.path.join(HERE, 'lab_run.sh'), '--tag', f'slot{slot}', '--cpus', str(cpus),
                '--domain', str(DOMAIN_BASE + slot), '--partition', f'wesley_{slot}',
                '--mount', f'{SLOTS_DIR}/slot{slot}:{scratch}',
                '--env', f'MDC_ACADOS_ROOT={scratch}', '--env', f'MDC_LAUNCH_CWD={scratch}',
                '--env', f'MDC_RESULTS_ROOT={scratch}/results', '--workdir', CTR_REPO, '--',
                'bash', '-c', self.warm_script(scratch), 'warm',
                'python3', 'tools/run_experiment.py', rel, '--run-dir', self.ctr_dir(slot),
                '--git-state-file', f'{scratch}/git_state.json', '--gz-nice', str(gz_nice)]


def reserve(configs, date, dry):
    """Jobs with run ids taken on the laptop; the directories are created unless dry."""
    from utility_objects.run_context import results_root
    root = results_root()
    rid = next_run_id(root)
    jobs = []
    for path, cfg in configs:
        local = os.path.join(root, date, f'R{rid:04d}_sim_gz_{cfg.name}')
        if not dry:
            os.makedirs(local)                                  # an existing dir is a collision
            with open(os.path.join(local, 'RESERVED.txt'), 'w') as fh:
                fh.write(f'reserved by tools/remote/lab_batch.py {datetime.datetime.now():%Y-%m-%dT%H:%M:%S} '
                         f'for {os.path.relpath(path, REPO)}; removed when the lab result arrives\n')
        jobs.append(Job(path, cfg, f'R{rid:04d}', local, date))
        # rescan after each reservation, so a run another process allocated meanwhile is skipped
        rid = next_run_id(root) if not dry else rid + 1
    return jobs


def fetch(job, slot):
    """rsync the staged result into the reserved directory; True if a manifest arrived."""
    src = f'{LAB}:{job.host_dir(slot)}/'
    r = subprocess.run(['rsync', '-az', '-e', 'ssh -o BatchMode=yes', src, job.local_dir + '/'],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        job.note = f'rsync failed: {r.stdout.decode(errors="replace").strip()[-200:]}'
        return False
    mf = os.path.join(job.local_dir, 'manifest.json')
    if not os.path.exists(mf):
        job.note = 'no manifest came back'
        return False
    with open(mf) as fh:
        man = json.load(fh)
    man['lab'] = {'host': LAB, 'slot': slot, 'container': f'wesley_slot{slot}',
                  'ros_domain_id': DOMAIN_BASE + slot, 'gz_partition': f'wesley_{slot}',
                  'container_run_dir': man.get('run_dir')}
    man['run_dir'] = job.local_dir
    with open(mf, 'w') as fh:
        json.dump(man, fh, indent=2, sort_keys=True, default=str)
    job.passed = man.get('passed')
    os.remove(os.path.join(job.local_dir, 'RESERVED.txt'))
    lab(f'rm -rf {shlex.quote(job.host_dir(slot))}', check=False)
    return True


def worker(slot, jobs_q, cpus, gz_nice, active, lock):
    while True:
        try:
            job = jobs_q.get_nowait()
        except queue.Empty:
            return
        job.slot = slot
        t0 = time.time()
        print(f'[slot {slot}] {job.rid} {job.cfg.name}: start', flush=True)
        log = os.path.join(job.local_dir, 'lab_console.log')
        with open(log, 'w') as fh:
            cmd = job.command(slot, cpus, gz_nice)
            fh.write('$ ' + ' '.join(shlex.quote(c) for c in cmd) + '\n\n')
            fh.flush()
            p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
            with lock:
                active[slot] = p
            job.rc = p.wait()
            with lock:
                active.pop(slot, None)
        job.wall_s = time.time() - t0
        got = fetch(job, slot)
        print(f'[slot {slot}] {job.rid} {job.cfg.name}: exit {job.rc}, {job.wall_s / 60:.1f} min, '
              f'{"PASS" if job.passed else "FAIL" if got else "NO RESULT"} {job.note}', flush=True)


# ── main ─────────────────────────────────────────────────────────────────────

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config', nargs='+', help='experiment YAMLs inside this repo')
    ap.add_argument('--slots', type=int, default=DEFAULT_SLOTS, help='at most this many in parallel (<= 6)')
    ap.add_argument('--cpus', type=float, default=4, help='cores per container (3-4)')
    ap.add_argument('--gz-nice', type=int, default=10)
    ap.add_argument('--dry-run', action='store_true',
                    help='check, read the lab load and print the plan; reserve, sync and run nothing')
    ap.add_argument('--no-sync', action='store_true', help='skip lab_sync.sh (tree already there)')
    ap.add_argument('--no-build', action='store_true', help='skip the colcon build on the lab')
    a = ap.parse_args(argv)
    sys.stdout.reconfigure(line_buffering=True)     # keep order with the sync/build output
    if not 3 <= a.cpus <= 4:
        ap.error('--cpus must be 3-4')

    configs = []
    for path in a.config:
        path = os.path.abspath(path)
        if not path.startswith(REPO + os.sep):
            ap.error(f'{path} is not inside {REPO}: the lab only has the synced tree')
        configs.append((path, ExperimentConfig.from_yaml(path)))
    r = subprocess.run([sys.executable, os.path.join(REPO, 'tools', 'run_experiment.py'), '--check',
                        *[p for p, _ in configs]], cwd=REPO)
    if r.returncode != 0:
        raise SystemExit('config check failed; nothing reserved or run')

    load = lab_load()
    # ~/mdc and its install are one tree mounted RW into every slot: syncing or building
    # under a live run changes its code mid-flight and makes its manifest wrong.
    rebuild = not (a.no_sync and a.no_build)
    if load['slots_in_use'] and rebuild:
        msg = (f'wesley_slot{load["slots_in_use"]} still running on the shared tree: wait for them, '
               f'or pass --no-sync --no-build to run beside them on the tree they use')
        if not a.dry_run:
            raise SystemExit(msg)
        print('would refuse:', msg)
    slots = pick_slots(load, a.slots, a.cpus, len(configs))
    print(f'lab: {load["ncpu"]} cores, busy {load["busy"]:.1f} (others {load["others"]:.1f}, '
          f'wesley_* {load["ours"]:.1f}), slots already up {load["slots_in_use"] or "none"} '
          f'-> {len(slots)} slot(s) {slots} at {a.cpus:g} cpus')
    if not slots:
        raise SystemExit('no free slot within the core budget; try later')

    date = time.strftime('%Y-%m-%d')
    if not a.dry_run and rebuild:
        up = slots_up()
        if up:
            raise SystemExit(f'{up} started meanwhile; nothing synced, reserved or run')
        if not a.no_sync:
            subprocess.run([os.path.join(HERE, 'lab_sync.sh')], check=True)
        if not a.no_build:
            subprocess.run([os.path.join(HERE, 'lab_build.sh'), '--ws'], check=True)
    jobs = reserve(configs, date, a.dry_run)
    gs = laptop_git_state()              # after the sync: the tree the runs got
    print(f'laptop tree: {gs["git_branch"]} {gs["git_sha"][:10]}, {gs["dirty_files"]} dirty, '
          f'diff {len(gs["diff"]) / 1024:.0f} kB' + (f', untracked skipped {gs["untracked_skipped"]}'
                                                        if gs['untracked_skipped'] else ''))
    for k, job in enumerate(jobs):
        s = slots[k % len(slots)]
        print(f'  {job.rid}  {os.path.relpath(job.cfg_path, REPO)}  -> {os.path.relpath(job.local_dir, REPO)}')
        if a.dry_run:
            print('      e.g. slot', s, ':', ' '.join(shlex.quote(c) for c in job.command(s, a.cpus, a.gz_nice)))
    if a.dry_run:
        print('dry run: nothing reserved, synced or started')
        return 0

    state = json.dumps(gs)
    for s in slots:
        # a network left by a crashed run of this slot blocks lab_run.sh; the slot is free (checked)
        subprocess.run(['ssh', '-o', 'BatchMode=yes', LAB,
                        f'mkdir -p {SLOTS_DIR}/slot{s} && cat > {SLOTS_DIR}/slot{s}/git_state.json && '
                        f'(docker network rm wesley_slot{s} >/dev/null 2>&1 || true)'],
                       input=state.encode(), check=True)

    jobs_q = queue.Queue()
    for job in jobs:
        jobs_q.put(job)
    active, lock = {}, threading.Lock()

    def stop_ours(signum, frame):
        with lock:
            mine = list(active)
        print(f'\ninterrupted: removing this batch\'s containers {[f"wesley_slot{s}" for s in mine]}')
        for s in mine:
            lab(f'docker rm -f wesley_slot{s} >/dev/null 2>&1; docker network rm wesley_slot{s} >/dev/null 2>&1',
                check=False)
        raise SystemExit(130)
    signal.signal(signal.SIGINT, stop_ours)
    signal.signal(signal.SIGTERM, stop_ours)

    threads = [threading.Thread(target=worker, args=(s, jobs_q, a.cpus, a.gz_nice, active, lock), daemon=True)
               for s in slots]
    t0 = time.time()
    for k, th in enumerate(threads):
        th.start()
        if k + 1 < len(threads):
            time.sleep(20)          # stagger the solver builds and Gazebo start-ups
    while any(th.is_alive() for th in threads):
        time.sleep(1.0)

    print(f'\n=== lab batch: {len(jobs)} run(s) in {(time.time() - t0) / 60:.1f} min on {len(slots)} slot(s) ===')
    print('| run | config | slot | exit | wall min | result | dir |')
    print('|---|---|---|---|---|---|---|')
    ok = True
    for job in jobs:
        res = 'PASS' if job.passed else ('FAIL' if job.passed is False else f'NO RESULT {job.note}'.strip())
        ok = ok and bool(job.passed)
        wall = f'{job.wall_s / 60:.1f}' if job.wall_s else '-'
        print(f'| {job.rid} | {job.cfg.name} | {job.slot} | {job.rc} | {wall} | {res} | '
              f'{os.path.relpath(job.local_dir, REPO)} |')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
