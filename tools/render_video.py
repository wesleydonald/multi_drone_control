#!/usr/bin/env python3
"""
tools/render_video.py -- render a recorded Gazebo run to mp4, one file per camera, in sim time.

    tools/render_video.py results/<date>/R####_sim_gz_<name>            # every recorded camera
    tools/render_video.py <run_dir> --camera wide --start 20 --end 80   # run-time seconds
    tools/render_video.py <run_dir> --camera ring --view ring:2.2,-2.1,3.1:0,0.05,1.7 --out-dir /tmp/v

The run must have been flown with recording on (run_experiment.py --record, or `record:` in the
config): <run>/gz_record/state.tlog plus <run>/record.json. This replays the log in a headless
Gazebo server (its own GZ_PARTITION) with the Sensors system loaded, so the camera models the
recording world carries render with ogre2 on the GPU (EGL); every frame is stamped in sim time
and written at its slot, 30 frames per sim second by default, played back at 1x sim speed.
The replay is paused whenever the encoder falls behind, so a slow machine only takes longer.
--view re-aims a recorded camera on a copy of the log, so a wrong framing never costs a re-fly.
Never run it while a live run or the rig stack is up on the same machine.
"""
import argparse
import glob
import json
import math
import os
import queue
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

SERVER_CONFIG = '''<server_config><plugins>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system"
          name="gz::sim::systems::UserCommands"/>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-sensors-system"
          name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
    <background_color>0.6 0.8 1.0 1</background_color>
    <ambient_light>0.5 0.5 0.5 1</ambient_light>
  </plugin>
</plugins></server_config>
'''


def log_span(tlog):
    """(first, last) sim time in the state log, seconds (gz-transport logs stamp in sim time)."""
    db = sqlite3.connect(f'file:{tlog}?mode=ro', uri=True)
    lo, hi = db.execute('SELECT MIN(time_recv), MAX(time_recv) FROM messages').fetchone()
    db.close()
    return lo * 1e-9, hi * 1e-9


def fnv1a64(text):
    """gz-sim's component type id: FNV-1a 64 of the registered component name."""
    h = 0xcbf29ce484222325
    for b in text.encode():
        h = ((h ^ b) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def repose_log(src_dir, dst_dir, poses):
    """Copy the log with camera models moved ({name: 'x y z roll pitch yaw'}). A static model's
    pose is logged once, as text, in the first state message; the field of view stays as recorded."""
    from gz.msgs10.serialized_map_pb2 import SerializedStateMap
    os.makedirs(dst_dir, exist_ok=True)
    shutil.copy2(os.path.join(src_dir, 'state.tlog'), os.path.join(dst_dir, 'state.tlog'))
    db = sqlite3.connect(os.path.join(dst_dir, 'state.tlog'))
    name_t, pose_t = fnv1a64('gz_sim_components.Name'), fnv1a64('gz_sim_components.Pose')
    mid, blob = db.execute(
        "SELECT m.id, m.message FROM messages m JOIN topics t ON m.topic_id = t.id "
        "WHERE t.name LIKE '%/changed_state' ORDER BY m.time_recv, m.id LIMIT 1").fetchone()
    state = SerializedStateMap()
    state.ParseFromString(blob)
    done = set()
    for ent in state.entities.values():
        comps = list(ent.components.values())
        name = next((c.component.decode() for c in comps if c.type == name_t), None)
        model = name[len('mdc_rec_'):] if name and name.startswith('mdc_rec_') else None
        if model in poses:
            for c in comps:
                if c.type == pose_t:
                    c.component = poses[model].encode()
                    done.add(model)
    missing = set(poses) - done
    if missing:
        raise SystemExit(f'--view: no camera model {sorted(missing)} in the first logged state')
    db.execute('UPDATE messages SET message = ? WHERE id = ?', (state.SerializeToString(), mid))
    db.commit()
    db.close()


def parse_view(text):
    """'name:x,y,z:lx,ly,lz' -> (name, 'x y z roll pitch yaw')."""
    sys.path.insert(0, HERE)
    from experiment.record import camera_pose
    try:
        name, pos, look = text.split(':')
        pos, look = [float(v) for v in pos.split(',')], [float(v) for v in look.split(',')]
        assert len(pos) == 3 and len(look) == 3
    except (ValueError, AssertionError):
        raise SystemExit(f'--view {text!r}: expected name:x,y,z:look_x,look_y,look_z')
    return name, ' '.join(f'{v:.6g}' for v in camera_pose(pos, look)), {'pos': pos, 'look_at': look}


class Camera:
    """One camera's frames -> ffmpeg. Frames are placed by their sim stamp: a missing slot repeats
    the previous frame (counted), so the file always plays at 1x sim time."""

    def __init__(self, name, out, size, fps, t0, t1, crf):
        self.name, self.out, self.fps, self.t0, self.t1 = name, out, fps, t0, t1
        self.w, self.h = size
        self.q = queue.Queue()
        self.next_k, self.last, self.repeats, self.frames = 0, None, 0, 0
        self.n_total = int(math.floor((t1 - t0) * fps + 1e-6)) + 1
        self.done = threading.Event()
        self.error = None
        self.ff = subprocess.Popen(
            ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
             '-pix_fmt', 'rgb24', '-s', f'{self.w}x{self.h}', '-r', str(fps), '-i', '-',
             '-c:v', 'libx264', '-preset', 'medium', '-crf', str(crf), '-pix_fmt', 'yuv420p',
             '-movflags', '+faststart', out],
            stdin=subprocess.PIPE)
        self.writer = threading.Thread(target=self._write, daemon=True)
        self.writer.start()

    def on_image(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if self.done.is_set() or t < self.t0 - 0.5 / self.fps:
            return
        if (msg.width, msg.height) != (self.w, self.h):
            self.error = f'{self.name}: image {msg.width}x{msg.height}, expected {self.w}x{self.h}'
            self.done.set()
            return
        self.q.put((t, msg.data))

    def _write(self):
        try:
            while True:
                item = self.q.get()
                if item is None:
                    return
                t, data = item
                k = int(round((t - self.t0) * self.fps))
                if k < self.next_k:
                    continue                       # a second frame for a slot already written
                if self.last is not None:
                    while self.next_k < min(k, self.n_total):
                        self.ff.stdin.write(self.last)
                        self.next_k += 1
                        self.repeats += 1
                if self.next_k >= self.n_total:
                    self.done.set()
                    continue
                self.ff.stdin.write(data)
                self.last, self.next_k = data, self.next_k + 1
                self.frames += 1
                if self.next_k >= self.n_total:
                    self.done.set()
        except (BrokenPipeError, OSError) as e:
            self.error = f'{self.name}: ffmpeg: {e}'
            self.done.set()

    def close(self):
        self.done.set()
        self.q.put(None)
        self.writer.join(timeout=600)
        try:
            self.ff.stdin.close()
        except OSError:
            pass
        return self.ff.wait(timeout=600) == 0 and self.error is None


def request(node, service, req, rep_type, timeout_ms=3000):
    try:
        ok, _ = node.request(service, req, type(req), rep_type, timeout_ms)
        return ok
    except Exception:                                  # noqa: BLE001
        return False


def render(run_dir, cameras=None, start=None, end=None, crf=18, out_dir=None, verbose=False,
           views=()):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.log_playback_control_pb2 import LogPlaybackControl
    from gz.transport13 import Node

    run_dir = os.path.abspath(run_dir)
    rec_dir = os.path.join(run_dir, 'gz_record')
    tlog = os.path.join(rec_dir, 'state.tlog')
    with open(os.path.join(run_dir, 'record.json')) as fh:
        spec = json.load(fh)
    if not os.path.exists(tlog):
        raise SystemExit(f'no state log: {tlog}')
    names = cameras or list(spec['cameras'])
    unknown = [c for c in names if c not in spec['cameras']]
    if unknown:
        raise SystemExit(f'camera(s) {unknown} not recorded; recorded: {list(spec["cameras"])}')
    fps = int(spec['fps'])
    lo, hi = log_span(tlog)
    # run time (run.csv's t) -> log sim time: t_ready_sim is run t = 0
    t_ready = spec.get('t_ready_sim')
    t_last = spec.get('t_last_row')
    if t_ready is None:
        t_ready = lo
    t0 = t_ready + (start if start is not None else -float(spec.get('lead_s', 1.0)))
    t1 = (t_ready + end) if end is not None else (
        t_ready + t_last + float(spec.get('tail_s', 1.0)) if t_last is not None else hi)
    t0, t1 = max(t0, lo), min(t1, hi - 2.0 / fps)     # the last slot well inside the log
    if t1 <= t0:
        raise SystemExit(f'nothing to render: [{t0:.2f}, {t1:.2f}] s of a log [{lo:.2f}, {hi:.2f}]')
    out_dir = out_dir or os.path.join(run_dir, 'video')
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.basename(run_dir).split('_')[0]

    tmp = tempfile.mkdtemp(prefix='render_video_', dir=out_dir)
    moved = {}
    for v in views:
        name, pose, where = parse_view(v)
        if name not in spec['cameras']:
            raise SystemExit(f'--view {name}: not recorded; recorded: {list(spec["cameras"])}')
        moved[name] = pose
        spec['cameras'][name].update(where)
    if moved:
        rec_dir = os.path.join(tmp, 'gz_record')
        repose_log(os.path.join(run_dir, 'gz_record'), rec_dir, moved)
        print(f'  moved {sorted(moved)} (field of view as recorded)')
    cfg_path = os.path.join(tmp, 'server.config')
    with open(cfg_path, 'w') as fh:
        fh.write(SERVER_CONFIG)
    env = dict(os.environ)
    partition = f'mdc_render_{os.getpid()}'
    env['GZ_PARTITION'] = partition
    os.environ['GZ_PARTITION'] = partition             # this process's own transport node
    env['GZ_SIM_SERVER_CONFIG_PATH'] = cfg_path
    assets = os.path.join(REPO, 'simulation_assets')
    env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [assets, os.path.join(assets, 'models'), env.get('GZ_SIM_RESOURCE_PATH', '')]).strip(os.pathsep)
    log_path = os.path.join(out_dir, 'render_gazebo.log')
    gz_log = open(log_path, 'w')
    cmd = ['gz', 'sim', '-s', '--headless-rendering', '-v', '4' if verbose else '2',
           '--playback', rec_dir]
    gz_log.write('$ ' + ' '.join(cmd) + '\n\n')
    gz_log.flush()
    print(f'render {run_dir}\n  cameras {names}, sim [{t0:.2f}, {t1:.2f}] s of the log '
          f'(run t {t0 - t_ready:+.2f} .. {t1 - t_ready:+.2f}), {fps} fps')
    gz = subprocess.Popen(cmd, cwd=assets, env=env, stdout=gz_log, stderr=subprocess.STDOUT,
                          start_new_session=True)
    cams, ok = {}, False
    wall0 = time.time()
    try:
        node = Node()
        control = None
        for _ in range(120):                          # the playback world is 'default'; find it
            svcs = [s for s in node.service_list() if s.endswith('/playback/control')]
            if svcs:
                control = svcs[0]
                break
            if gz.poll() is not None:
                raise RuntimeError(f'gz exited ({gz.returncode}); see {log_path}')
            time.sleep(0.5)
        if control is None:
            raise RuntimeError(f'no playback control service after 60 s; see {log_path}')

        def playback(**kw):
            req = LogPlaybackControl()
            for k, v in kw.items():
                if k == 'seek':
                    req.seek.sec, req.seek.nsec = int(v), int(round((v - int(v)) * 1e9))
                else:
                    setattr(req, k, v)
            return request(node, control, req, Boolean)

        # cameras render only while subscribed, so seek there first (paused, nothing drawn)
        if t0 - lo > 0.5 and not playback(pause=True, seek=max(t0 - 0.2, lo)):
            print('  !! seek refused: replaying from the start (cameras idle until subscribed)')
        for name in names:
            c = spec['cameras'][name]
            out = os.path.join(out_dir, f'{base}_{name}.mp4')
            cams[name] = Camera(name, out, c['size'], fps, t0, t1, crf)
            node.subscribe(Image, spec['topic_prefix'] + name, cams[name].on_image)
        time.sleep(2.0)                               # subscriptions reach the server
        if not playback(pause=False):
            raise RuntimeError('play request refused')
        paused, last_report, stalled_since = False, 0.0, None
        progress = {n: -1 for n in names}
        while not all(c.done.is_set() for c in cams.values()):
            time.sleep(0.05)
            if gz.poll() is not None:
                raise RuntimeError(f'gz exited mid-render ({gz.returncode}); see {log_path}')
            backlog = max(c.q.qsize() for c in cams.values())
            if backlog > 12 and not paused:
                paused = playback(pause=True)
            elif backlog < 3 and paused:
                paused = not playback(pause=False)
            now_k = {n: c.next_k for n, c in cams.items()}
            if now_k != progress:
                progress, stalled_since = now_k, None
            elif not paused:
                stalled_since = stalled_since or time.time()
                if time.time() - stalled_since > 60:
                    # the log ended (playback pauses itself) or the cameras stopped publishing
                    print(f'  !! no frame for 60 s wall at slot {now_k}; stopping')
                    break
            if time.time() - last_report > 10:
                last_report = time.time()
                k = min(now_k.values())
                n = next(iter(cams.values())).n_total
                print(f'  {k}/{n} frames ({100 * k / n:.0f} %), {time.time() - wall0:.0f} s wall',
                      flush=True)
        ok = True
    finally:
        results = {n: c.close() for n, c in cams.items()}
        try:
            os.killpg(os.getpgid(gz.pid), signal.SIGINT)
            gz.wait(timeout=20)
        except Exception:                             # noqa: BLE001
            try:
                os.killpg(os.getpgid(gz.pid), signal.SIGKILL)
            except Exception:                         # noqa: BLE001
                pass
        gz_log.close()
        shutil.rmtree(tmp, ignore_errors=True)
    summary = {}
    for n, c in cams.items():
        good = ok and results.get(n) and c.next_k >= c.n_total
        summary[n] = {'file': c.out, 'frames': c.next_k, 'expected': c.n_total, 'drawn': c.frames,
                      'repeated': c.repeats, 'ok': bool(good), 'error': c.error}
        print(f'  [{"OK" if good else "FAIL"}] {n}: {c.out}  {c.next_k}/{c.n_total} frames '
              f'({c.repeats} repeated)' + (f'  {c.error}' if c.error else ''))
    print(f'  {time.time() - wall0:.0f} s wall')
    with open(os.path.join(out_dir, 'render.json'), 'w') as fh:
        json.dump({'t0_sim': t0, 't1_sim': t1, 'fps': fps, 'wall_s': time.time() - wall0,
                   'cameras': summary, 'views': {n: spec['cameras'][n] for n in names}}, fh, indent=1)
    return all(s['ok'] for s in summary.values()), summary


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--camera', action='append', help='camera name (repeatable); default: all recorded')
    ap.add_argument('--start', type=float, help='run time (s) to start at; default: lead_s before t=0')
    ap.add_argument('--end', type=float, help='run time (s) to end at; default: the last row + tail_s')
    ap.add_argument('--crf', type=int, default=18, help='x264 quality (lower = better, larger)')
    ap.add_argument('--out-dir', help='default <run_dir>/video')
    ap.add_argument('--view', action='append', default=[], metavar='NAME:X,Y,Z:LX,LY,LZ',
                    help='re-aim a recorded camera (position : look-at point, m) for this render')
    ap.add_argument('-v', '--verbose', action='store_true', help='gz -v 4 in render_gazebo.log')
    a = ap.parse_args()
    if not glob.glob(os.path.join(a.run_dir, 'gz_record', 'state.tlog')):
        ap.error(f'{a.run_dir}: no gz_record/state.tlog (fly it with run_experiment.py --record)')
    ok, _ = render(a.run_dir, a.camera, a.start, a.end, a.crf, a.out_dir, a.verbose, a.view)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
