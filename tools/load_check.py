#!/usr/bin/env python3
"""
tools/load_check.py -- the F1 load bars (rig card 2026-10-21_combined_m1_rig, section 5.1) from a bag or live.

    python3 tools/load_check.py results/rig/2026-10-13/desk_c --log results/rig/2026-10-13/desk_c.log
    python3 tools/load_check.py BAG --log T2.log --flight-window     # his TAKEOFF to the switch or the take-back
    python3 tools/load_check.py BAG --log T2.log --flight-window --end $E   # the props-off load test: his TAKEOFF
                                                                     # to E=$(date +%s.%N), typed before the kill path
    python3 tools/load_check.py BAG --log T2.log --start 60 --end 660   # s from the bag's first message
    python3 tools/load_check.py --live 600 --log T2.log              # subscribe for 600 s instead of a bag

The bag is the one tools/rig_flight.sh records (partner_contact:=true adds his topics); the log is that flight's
tee'd T2 console (mode:=attach puts his nodes in the same launch, so one log has both stacks' lines).

Per topic: message count, rate, the longest inter-arrival gap (and when), the 99th percentile gap and how many gaps
exceed the bar. Arrival time is the recorder's receive stamp (the bag) or this node's (--live): a starved recorder
inflates the gaps, so a FAIL is re-read before it is believed, a PASS stands.

The bars (frozen in the card; the topic names as the code publishes them in real_control_launch.py mode:=attach):
  < 0.15 s  /drone_3/ELRSCommand_tejen   his MPC (tejen_mission partner_mission.launch.py remaps it there)
            /drone_3/ELRSCommand         QUAD4's radio input, the mux's output (drone_magnet elrs_mux)
            /drone_3/ELRSCommand_diss    our drone-3 tracker, the mux's other input (bringup graph_attach)
            /drone_0..2/ELRSCommand      the carriers' radio inputs. The card names "each /drone_i/ELRSCommand_diss",
                                         but in mode:=attach only drone 3 has a _diss topic: the carriers' trackers
                                         publish straight onto their radio's topic. Scored here as the bar's intent
                                         (a radio disarms after 0.5 s without a command); --no-carriers drops them.
  < 0.3 s   /join_planner/reference      his planner's reference to his MPC (his MPC latches a fault on a stale one)
  0 lines   "External-reference fault latched" in the log (his MPC, tejen_mpc main.py)
  0 lines   "Pose timeout (" in the log (our trackers' watchdog; the card's "zero pose timeouts")
  0 lines   "disarm triggered - disarming drone" (his MPC: the only line its 0.25 s pose watchdog prints)

Window: by default each topic is scored from its own first to its last message (start-up and shut-down are not
gaps). With --start/--end or --flight-window the window is fixed and its edges count: a stream silent across an
edge is a gap. Exit 0 = every bar passes, 1 = a bar fails, 2 = incomplete (no log given, or no window found).
"""
import argparse
import datetime
import math
import re
import sys
import time

import numpy as np

FAST_BAR_S = 0.15
REF_BAR_S = 0.30
HIS_MPC = '/drone_3/ELRSCommand_tejen'
QUAD4_RADIO = '/drone_3/ELRSCommand'
OUR_QUAD4 = '/drone_3/ELRSCommand_diss'
REFERENCE = '/join_planner/reference'
ELRS_TYPE = 'interfaces/msg/ELRSCommand'
REF_TYPE = 'trajectory_msgs/msg/MultiDOFJointTrajectory'
MOCAP_TOPICS = ('/drone_0/motion_capture_state', '/drone_1/motion_capture_state', '/drone_2/motion_capture_state',
                '/drone_3/motion_capture_state', '/payload/motion_capture_state', '/magnet_tip_pose',
                '/pickup/motion_capture_state')

# (pattern, label, bar): a bar of 0 means any line fails; None is reported only
LOG_CHECKS = (
    ('External-reference fault latched', "his MPC 'External-reference fault latched'", 0),
    ('Pose timeout (', "our trackers' 'Pose timeout'", 0),
    # his MPC's 0.25 s pose watchdog logs only this line (it also ends a lost reference): his pose timeout
    ('disarm triggered - disarming drone', "his MPC's own disarm (his pose timeout)", 0),
    ('No message received for 0.1 seconds', "a radio's silence disarm (elrs_interface; rig only)", None),
)
STAMP_RE = re.compile(r'\[(\d{9,}\.\d+)\]')
HIS_TAKEOFF_RE = re.compile(r'\[(\d{9,}\.\d+)\] \[tejen\.tejen_mpc\]: Takeoff requested')
SWITCH_RE = re.compile(r'\[(\d{9,}\.\d+)\] \[elrs_mux[^\]]*\]: \[elrs_mux\] (?:-> DISSIPATIVE tracker|TAKE-BACK:)')


def f1_bars(carriers=3):
    """[(topic, bar_s, what)] in report order."""
    bars = [(HIS_MPC, FAST_BAR_S, 'his MPC'),
            (QUAD4_RADIO, FAST_BAR_S, "QUAD4's radio input (mux out)"),
            (OUR_QUAD4, FAST_BAR_S, 'our drone-3 tracker (mux in)')]
    bars += [(f'/drone_{i}/ELRSCommand', FAST_BAR_S, f'carrier {i + 1} radio input (card: _diss)')
             for i in range(carriers)]
    bars.append((REFERENCE, REF_BAR_S, "his planner's reference"))
    return bars


def topic_stats(times, bar=None, window=None):
    """Gap statistics of one topic's arrival times (s).

    window None: from the first to the last message. window (a, b): only messages inside it, and the edges a -> first
    and last -> b count as gaps. Returns a dict; 'verdict' is PASS/FAIL against bar (None = reported only)."""
    t = np.sort(np.asarray(times, dtype=float))
    if window is not None:
        a, b = window
        t = t[(t >= a) & (t <= b)]
        edges = np.concatenate(([a], t, [b]))
        gaps = np.diff(edges)
        span = b - a
        rate = len(t) / span if span > 0 else float('nan')
    else:
        gaps = np.diff(t)
        span = t[-1] - t[0] if len(t) > 1 else 0.0
        rate = (len(t) - 1) / span if span > 0 else float('nan')
    out = {'n': int(len(t)), 'span_s': float(span), 'rate_hz': float(rate),
           'first': float(t[0]) if len(t) else float('nan'), 'last': float(t[-1]) if len(t) else float('nan')}
    if len(gaps) and (len(t) > 1 or window is not None):
        k = int(np.argmax(gaps))
        out['max_gap_s'] = float(gaps[k])
        # the gap's start: the message before it (or the window start)
        starts = np.concatenate(([window[0]], t)) if window is not None else t[:-1]
        out['max_gap_at'] = float(starts[k])
        out['p99_gap_s'] = float(np.percentile(gaps, 99))
        out['over_bar'] = int(np.sum(gaps >= bar)) if bar is not None else 0
    else:
        out.update(max_gap_s=float('nan'), max_gap_at=float('nan'), p99_gap_s=float('nan'), over_bar=0)
    if bar is None:
        out['verdict'] = ''
    elif out['n'] == 0 or (window is None and out['n'] < 2):
        out['verdict'] = 'FAIL'           # nothing to score: not published, or not recorded
        out['why'] = 'no messages' if out['n'] == 0 else 'one message'
    else:
        out['verdict'] = 'PASS' if out['max_gap_s'] < bar else 'FAIL'
    return out


def scan_log(text, window=None):
    """[(label, bar, count, first matching lines)] of LOG_CHECKS in a console log; window (a, b) counts only lines
    stamped inside it (lines without a stamp always count)."""
    out = []
    lines = text.splitlines()
    for pattern, label, bar in LOG_CHECKS:
        hits = []
        for ln in lines:
            if pattern not in ln:
                continue
            if window is not None:
                m = STAMP_RE.search(ln)
                if m and not (window[0] <= float(m.group(1)) <= window[1]):
                    continue
            hits.append(ln.strip())
        out.append((label, bar, len(hits), hits[:3]))
    return out


def flight_window(text):
    """(start, end, note) from a T2 log: his TAKEOFF ('Takeoff requested' from tejen.tejen_mpc) to the mux's switch
    to our tracker or its take-back; end None when neither came (then the caller ends at the recording's end)."""
    m = HIS_TAKEOFF_RE.search(text)
    if not m:
        return None, None, 'no "Takeoff requested" from tejen.tejen_mpc in the log'
    a = float(m.group(1))
    for s in SWITCH_RE.finditer(text):
        if float(s.group(1)) >= a:
            what = 'switch' if 'DISSIPATIVE' in s.group(0) else 'take-back'
            return a, float(s.group(1)), f'his TAKEOFF to the mux {what}'
    return a, None, 'his TAKEOFF to the end of the recording (no switch or take-back in the log)'


def read_bag(path, topics):
    """({topic: [receive times s]}, {topic: last message bytes}, {topic: type}, (t0, t1)) for the given topics."""
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    times, last = {t: [] for t in wanted}, {}
    t_all = [math.inf, -math.inf]
    meta = r.get_metadata()
    t0, dur = _seconds(meta.starting_time), _seconds(meta.duration)
    if wanted:
        r.set_filter(rosbag2_py.StorageFilter(topics=wanted))
        while r.has_next():
            topic, data, ts = r.read_next()
            s = ts * 1e-9
            times[topic].append(s)
            last[topic] = data
            t_all[0], t_all[1] = min(t_all[0], s), max(t_all[1], s)
    if t0 is not None and dur is not None:
        t_all[0] = min(t_all[0], t0)
        t_all[1] = max(t_all[1], t0 + dur)
    return times, last, types, tuple(t_all)


def _seconds(v):
    """Seconds of a bag metadata time or duration (rclpy Time/Duration in this Humble build, datetime elsewhere)."""
    if hasattr(v, 'nanoseconds'):
        return v.nanoseconds * 1e-9
    if hasattr(v, 'timestamp'):
        return v.timestamp()
    if hasattr(v, 'total_seconds'):
        return v.total_seconds()
    return None


def record_live(topics, seconds):
    """Subscribe to the topics for `seconds` (raw, no deserialisation): ({topic: [times]}, {topic: last bytes},
    {topic: type}, (t0, t1)). Types come from the graph, so a topic must be advertised when this starts."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    node = rclpy.create_node('load_check')
    types = {}
    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end and len(types) < len(topics):
        types = {n: ts[0] for n, ts in node.get_topic_names_and_types() if n in topics and ts}
        rclpy.spin_once(node, timeout_sec=0.2)
    times, last = {t: [] for t in topics}, {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    def cb(topic):
        def f(data):
            times[topic].append(time.time())
            last[topic] = data
        return f
    for topic, typ in types.items():
        node.create_subscription(get_message(typ), topic, cb(topic), qos, raw=True)
    t0 = time.time()
    t_stop = time.monotonic() + seconds
    while time.monotonic() < t_stop:
        rclpy.spin_once(node, timeout_sec=0.05)
    t1 = time.time()
    node.destroy_node()
    rclpy.shutdown()
    return times, last, types, (t0, t1)


def last_armed(data, typ):
    """'armed' / 'disarmed' of the last ELRSCommand, '' for other types or when it cannot be read."""
    if typ != ELRS_TYPE or data is None:
        return ''
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        return 'armed' if deserialize_message(bytes(data), get_message(typ)).armed else 'disarmed'
    except Exception:
        return '?'


def clock(t):
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3] if math.isfinite(t) else '-'


def report(source, times, last, types, span, bars, also=(), window=None, window_note='', logs=()):
    """Print the table; return the exit code (0 pass, 1 fail, 2 incomplete)."""
    t0 = span[0]
    print(f'load_check: {source}  ({span[1] - span[0]:.1f} s, {clock(span[0])} -> {clock(span[1])})')
    if window is None:
        print('window: whole recording, each topic from its first to its last message')
    else:
        print(f'window: +{window[0] - t0:.1f} s -> +{window[1] - t0:.1f} s ({window[1] - window[0]:.1f} s; '
              f'{window_note}); its edges count as gaps')
    hdr = (f'{"topic":30s} {"bar s":>5s} {"n":>7s} {"rate Hz":>7s} {"max gap s":>9s} {"at +s":>7s} '
           f'{"p99 s":>6s} {">=bar":>5s} {"last":>8s}  verdict')
    print(hdr)
    print('-' * len(hdr))
    failed, rows = [], []
    for topic, bar, what in list(bars) + [(t, None, 'reported') for t in also]:
        s = topic_stats(times.get(topic, []), bar, window)
        if topic not in types and bar is not None:
            s['verdict'], s['why'] = 'FAIL', 'not in the bag / not advertised'
        if bar is None and s['n'] == 0:
            continue
        arm = last_armed(last.get(topic), types.get(topic))
        verdict = s['verdict'] + (f" ({s['why']})" if s.get('why') else '')
        print(f'{topic:30s} {"" if bar is None else f"{bar:.2f}":>5s} {s["n"]:7d} {s["rate_hz"]:7.1f} '
              f'{s["max_gap_s"]:9.3f} {s["max_gap_at"] - t0:7.1f} {s["p99_gap_s"]:6.3f} '
              f'{s["over_bar"] if bar is not None else "":>5} {arm:>8s}  {verdict}')
        rows.append((topic, s))
        if s['verdict'] == 'FAIL':
            failed.append(topic)
    incomplete = []
    if not logs:
        incomplete.append('no --log: the fault and pose-timeout lines are not checked')
    for path in logs:
        try:
            text = open(path, errors='replace').read()
        except OSError as e:
            incomplete.append(f'log {path}: {e}')
            continue
        print(f'log {path}' + ('' if window is None else ' (lines inside the window)'))
        for label, bar, count, first in scan_log(text, window):
            mark = '' if bar is None else ('PASS' if count <= bar else 'FAIL')
            print(f'  {count:4d}  {label}' + (f'  (bar {bar})  {mark}' if bar is not None else '  (reported)'))
            for ln in first:
                print(f'          {ln[:160]}')
            if mark == 'FAIL':
                failed.append(label)
    scored = len([b for b in bars])
    if failed:
        print(f'F1 load bars: FAIL ({len(failed)} failing: {", ".join(failed)})')
        return 1
    if incomplete:
        print('F1 load bars: INCOMPLETE (' + '; '.join(incomplete) + f'); the {scored} topic bars pass')
        return 2
    print(f'F1 load bars: PASS ({scored} topics and the log lines)')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', nargs='?', help='rosbag2 directory (tools/rig_flight.sh writes results/rig/<date>/<name>)')
    ap.add_argument('--live', type=float, metavar='S', help='subscribe for S seconds instead of reading a bag')
    ap.add_argument('--log', action='append', default=[], help="the flight's T2 log (repeatable)")
    ap.add_argument('--start', type=float, help='window start, s from the first message (or an epoch time)')
    ap.add_argument('--end', type=float, help='window end, s from the first message (or an epoch time)')
    ap.add_argument('--flight-window', action='store_true',
                    help="the card's scoring window from --log: his TAKEOFF to the mux switch or the take-back")
    ap.add_argument('--carriers', type=int, default=3, help='carrier radio streams scored (default 3)')
    ap.add_argument('--no-carriers', action='store_true', help="score only the card's literal topics")
    ap.add_argument('--mocap', action='store_true', help='also report the pose topics (no bar)')
    ap.add_argument('--also', action='append', default=[], help='another topic to report (no bar; repeatable)')
    a = ap.parse_args(argv)
    if (a.bag is None) == (a.live is None):
        ap.error('give a bag directory or --live S')
    bars = f1_bars(0 if a.no_carriers else a.carriers)
    also = list(a.also) + (list(MOCAP_TOPICS) if a.mocap else [])
    topics = [t for t, _, _ in bars] + also
    if a.live is not None:
        times, last, types, span = record_live(topics, a.live)
        source = f'live {a.live:.0f} s'
    else:
        times, last, types, span = read_bag(a.bag, topics)
        source = f'bag {a.bag}'
    if not math.isfinite(span[0]):
        print(f'load_check: {source}: none of the topics has a message')
        return 1
    window, note = None, ''
    # a typed bound is s from the first message, or an epoch time (`date +%s.%N` in T3)
    rel = lambda v: v if v > 1e9 else span[0] + v  # noqa: E731
    if a.flight_window:
        if not a.log:
            ap.error('--flight-window reads his TAKEOFF and the switch from --log')
        text = '\n'.join(open(p, errors='replace').read() for p in a.log)
        w0, w1, note = flight_window(text)
        if w0 is None:
            print(f'load_check: {note}: no flight window')
            return 2
        # --start / --end override either end (the load test: his TAKEOFF to the time typed before the kill path)
        if a.end is not None:
            w1, note = rel(a.end), note.split(' to ')[0] + ' to the typed end'
        window = (rel(a.start) if a.start is not None else w0, span[1] if w1 is None else w1)
    elif a.start is not None or a.end is not None:
        window = (rel(a.start) if a.start is not None else span[0], rel(a.end) if a.end is not None else span[1])
        note = 'typed'
    if window is not None and not window[1] > window[0]:
        ap.error(f'empty window {window}')
    return report(source, times, last, types, span, bars, also, window, note, a.log)


if __name__ == '__main__':
    sys.exit(main())
