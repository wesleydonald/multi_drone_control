#!/usr/bin/env python3
"""tools/console.py -- the rig console: is the fleet ready, who is in the air, and the flight's buttons.

    python3 tools/console.py                 # rig: four drones, QUAD4 flown by Tejen's stack
    python3 tools/console.py --no-partner    # our fleet alone
    python3 tools/console.py --twin          # the twin (no radios, so no FC or pack checks)

READY (top) replaces the fixed wait after the T2 launch: every node up, every body's mocap fresh, a fresh pack
voltage >= 24.0 V from every radio, every FC ready to arm (flight mode ending in '*', not '!ERR*'), no drone on its
side, no mux latched. Each reason it is not ready is listed, with what to do.
Buttons, in flight order: his ARM, ARM, TAKEOFF, his TAKEOFF, GIVE UP, LAND, and his DISARM once every FC is
disarmed. A button that cannot be used says why. SPACE is DISARM from anywhere in the window and is never blocked,
as in the RViz panel. The publishers live as long as the console, so every subscriber has matched long before a
press (14 Oct's takeoff race came from a fresh `ros2 topic pub` each time).
It reads only low-rate topics (/fleet/mocap_summary at 5 Hz, /drone_i/radio_health at 1 Hz, states and phases),
so it costs little next to the controllers. tools/fleet_monitor.py stays the read-only view.
"""
import argparse
import json
import sys
import threading
import time
from collections import deque

from fleet_monitor import ALARM_WORDS, explain

POSE_STALE_S = 0.25        # the rig trackers' pose timeout
SUMMARY_STALE_S = 1.0
HEALTH_STALE_S = 2.5       # radio_health comes once a second
MODE_STALE_S = 5.0         # QUAD1 sends a flight-mode frame only every 2-3 s
VOLT_STALE_S = 5.0
VOLT_MIN = 24.0            # the pre-flight rule, at rest
ERR_REPLUG_S = 20.0        # '!ERR*' this long is not the short block after a boot or a disarm
TILT_DOWN_DEG = 25.0       # the carriers rest at 11-13 deg on their rods; beyond this one is on its side (plan rev 1)
FC_TILT_OFF_DEG = 10.0     # QUAD1's FC read 60-84 deg while it sat at 12 (14 Oct)
AIRBORNE_M = 0.05          # above its rest height
REST_Z_DEFAULT = 0.07      # m: the rig's drones rest at 0.05-0.07 (14 Oct); used until a rest is seen
REST_LEARN_MAX_Z = 0.30    # m: rest heights are learned only below this (a console started mid-flight learns none)
TWIN_STALE_X = 3.0         # the twin runs at ~0.2x real time: its 5 Hz summary arrives about once a wall second
HIS_TAKEOFF_ACK_S = 2.0    # his planner must leave WAIT_FOR_TAKEOFF this soon after his TAKEOFF (Tejen, 15 Oct)
DISARM_REPEATS, DISARM_PERIOD_MS = 5, 100      # as the RViz panel: one message can be lost
FLEET_CMD, HIS_CMD, GIVE_UP = '/fleet/command', '/tejen/drone_command', '/join_planner/give_up'
HIS_NODES = ('/tejen/tejen_mpc', '/online_join_planner')
ARM_DONE = 'ARM sequence complete'
ARM_RESET = ('DISARM', 'EMERGENCY STOP', 'ARM FAILED', 'ARM REFUSED', 'TAKEOFF REFUSED', 'Disarm complete')


def quad(i):
    return f'QUAD{i + 1}'


def fc_mode_state(mode):
    """'armed', 'ready', 'blocked' or None from a Betaflight CRSF flight-mode string ('AIR', 'AIR*', '!ERR*')."""
    mode = (mode or '').strip()
    if not mode or mode == 'UNKNOWN':
        return None
    if mode.startswith('!'):
        return 'blocked'
    return 'ready' if mode.endswith('*') else 'armed'


def expected_nodes(n, partner, twin):
    """(what, accepted node names) the flight needs."""
    need = [('fleet manager', ('/fleet_manager',)), ('our planner', ('/dissipative_planner', '/mpc_planner')),
            ('fleet_viz (the mocap summary)', ('/fleet_viz',))]
    need += [(f'{quad(i)} tracker', (f'/tracker_{i}',)) for i in range(n)]
    if not twin:
        need.append(('mocap', ('/motion_capture_publisher',)))
        need += [(f'{quad(i)} radio', (f'/drone_{i}/elrs_interface',)) for i in range(n)]
    if partner is not None:
        need += [('his MPC', (HIS_NODES[0],)), ('his planner', (HIS_NODES[1],))]
    return need


class ConsoleState:
    """What the window shows, filled by ROS callbacks and read by the GUI timer (under self.lock)."""

    def __init__(self, n, partner=3, twin=False, small_angle=25.0):
        self.n, self.partner, self.twin, self.small_angle = n, partner, twin, small_angle
        self.lock = threading.Lock()
        self.manager = ''
        self.arm_complete = False
        self.phase = self.his_phase = ''
        self.proof = None
        self.contact = self.attached = None
        self.summary, self.summary_t = {}, None
        self.health, self.health_t, self.err_since = [None] * n, [None] * n, [None] * n
        self.tracker_armed = [None] * n
        self.his_armed = None
        self.mux = [''] * n
        self.rest_z = {}
        self.nodes = set()
        self.his_subs, self.his_matched, self.give_up_subs = set(), 0, 0
        self.alerts = deque(maxlen=200)      # (wall time, level, node, text, why)

    def on_manager(self, text):
        with self.lock:
            self.manager = text
            if ARM_DONE in text:
                self.arm_complete = True
            elif any(w in text for w in ARM_RESET):
                self.arm_complete = False

    def on_tracker_armed(self, i, armed):
        with self.lock:
            self.tracker_armed[i] = bool(armed)
            if not armed:
                self.arm_complete = False

    def on_his_armed(self, armed):
        with self.lock:
            self.his_armed = bool(armed)

    def on_health(self, i, text, now):
        try:
            h = json.loads(text)
        except ValueError:
            return
        with self.lock:
            self.health[i], self.health_t[i] = h, now
            fresh = h.get('mode_age_s') is not None and h['mode_age_s'] <= MODE_STALE_S
            if fresh and fc_mode_state(h.get('mode')) == 'blocked':
                self.err_since[i] = self.err_since[i] or now
            elif fresh:
                self.err_since[i] = None

    def on_summary(self, text, now):
        try:
            doc = json.loads(text)
        except ValueError:
            return
        with self.lock:
            self.summary, self.summary_t = doc, now
            for name, b in doc.get('bodies', {}).items():
                z, age = b.get('z'), b.get('age_s')
                i = int(name.split('_')[1]) if name.startswith('drone_') else None
                armed = i is not None and i < self.n and (
                    self.tracker_armed[i] is True or fc_mode_state((self.health[i] or {}).get('mode')) == 'armed')
                if z is not None and age is not None and age <= POSE_STALE_S and z < REST_LEARN_MAX_Z and not armed:
                    self.rest_z[name] = min(self.rest_z.get(name, z), z)

    def on_phase(self, text):
        with self.lock:
            self.phase = text

    def on_his_phase(self, text):
        with self.lock:
            self.his_phase = text

    def on_proof(self, text):
        try:
            p = json.loads(text)
        except ValueError:
            return
        with self.lock:
            self.proof = p

    def on_contact(self, value):
        with self.lock:
            self.contact = bool(value)

    def on_attached(self, value):
        with self.lock:
            self.attached = bool(value)

    def on_mux(self, i, text):
        with self.lock:
            self.mux[i] = text

    def on_graph(self, nodes, his_subs, his_matched, give_up_subs):
        with self.lock:
            self.nodes, self.his_subs, self.his_matched, self.give_up_subs = (
                set(nodes), set(his_subs), int(his_matched), int(give_up_subs))

    def on_log(self, level, node, text):
        if level < 30:                         # rcl_interfaces/Log: 30 WARN, 40 ERROR, 50 FATAL
            return
        with self.lock:
            self.alerts.appendleft((time.time(), 'ERROR' if level >= 40 else 'WARN', node, text, explain(text)))

    def snapshot(self):
        """A copy the GUI and the rules read without the lock."""
        with self.lock:
            s = ConsoleState.__new__(ConsoleState)
            s.__dict__.update({k: (list(v) if isinstance(v, (list, deque)) else
                                   dict(v) if isinstance(v, dict) else set(v) if isinstance(v, set) else v)
                               for k, v in self.__dict__.items() if k != 'lock'})
            return s


def bodies(st, now):
    """The summary's bodies, or None when the summary is stale."""
    if st.summary_t is None or now - st.summary_t > SUMMARY_STALE_S * (TWIN_STALE_X if st.twin else 1.0):
        return None
    return st.summary.get('bodies', {})


def fc_state(st, i, now):
    """The FC's state from its radio ('armed', 'ready', 'blocked', None when unknown)."""
    h, t = st.health[i], st.health_t[i]
    if h is None or t is None or now - t > HEALTH_STALE_S:
        return None
    if h.get('mode_age_s') is None or h['mode_age_s'] > MODE_STALE_S:
        return None
    return fc_mode_state(h.get('mode'))


def fc_check(st, i, now):
    """(ok, text): this drone's FC will take an ARM."""
    h, t = st.health[i], st.health_t[i]
    if h is None or t is None or now - t > HEALTH_STALE_S:
        return False, 'no radio_health (its radio node down?)'
    if not h.get('connected'):
        return False, f"radio port {h.get('port')} closed: reseat the USB"
    state = fc_state(st, i, now)
    if state is None:
        return False, 'no flight mode from the FC (pack in? receiver bound?)'
    if state == 'blocked':
        dur = now - st.err_since[i] if st.err_since[i] else 0.0
        return False, (f"FC blocks arming ('{h['mode']}' for {dur:.0f} s)"
                       + (': replug its pack' if dur >= ERR_REPLUG_S else ''))
    tilt, att_age = h.get('fc_tilt_deg'), h.get('attitude_age_s')
    if (state == 'ready' and tilt is not None and att_age is not None and att_age <= MODE_STALE_S
            and tilt > st.small_angle):
        return False, (f'FC reads {tilt:.0f} deg of tilt > small_angle {st.small_angle:.0f}: it will refuse ARM '
                       '(replug it)')
    if state == 'armed' and i != st.partner:
        return False, f"FC armed ('{h['mode']}') before the fleet ARM"
    return True, h['mode']


def pack_check(st, i, now):
    """(ok, text): a fresh pack voltage at or above the pre-flight rule."""
    h, t = st.health[i], st.health_t[i]
    if h is None or t is None or now - t > HEALTH_STALE_S:
        return False, 'no radio_health'
    v, age = h.get('voltage'), h.get('voltage_age_s')
    if v is None or age is None or age > VOLT_STALE_S:
        return False, 'no pack voltage from its radio (no voltage correction without it)'
    if v < VOLT_MIN:
        return False, f'pack {v:.2f} V (rule: >= {VOLT_MIN:.1f} V at rest)'
    return True, f'{v:.2f} V'


def readiness(st, now):
    """Why the fleet is not ready to arm; empty = READY."""
    why = []
    missing = [what for what, names in expected_nodes(st.n, st.partner, st.twin) if not st.nodes & set(names)]
    if missing:
        why.append('not running: ' + ', '.join(missing))
    bs = bodies(st, now)
    if bs is None:
        why.append('no /fleet/mocap_summary (fleet_viz)')
    else:
        for i in range(st.n):
            b = bs.get(f'drone_{i}') or {}
            if b.get('age_s') is None or b['age_s'] > POSE_STALE_S:
                why.append(f'{quad(i)}: no fresh mocap')
            elif (b.get('tilt_deg') or 0.0) > TILT_DOWN_DEG:
                why.append(f'{quad(i)}: on its side ({b["tilt_deg"]:.0f} deg in mocap)')
        r = bs.get('payload') or {}
        if r.get('age_s') is None or r['age_s'] > POSE_STALE_S:
            why.append('ring: no fresh mocap')
    if not st.twin:
        for i in range(st.n):
            for ok, text in (fc_check(st, i, now), pack_check(st, i, now)):
                if not ok:
                    why.append(f'{quad(i)}: {text}')
    why += [f'{quad(i)}: mux latched by an abort (relaunch T2)' for i in range(st.n) if st.mux[i] == 'latched']
    return why


def airborne(st, now):
    """[(drone index, height above its rest)] for the drones off the ground."""
    bs = bodies(st, now) or {}
    out = []
    for i in range(st.n):
        b, rest = bs.get(f'drone_{i}') or {}, st.rest_z.get(f'drone_{i}', REST_Z_DEFAULT)
        if b.get('z') is not None and b['z'] - rest > AIRBORNE_M:
            out.append((i, b['z'] - rest))
    return out


def manager_flying(st):
    return st.manager.startswith('TAKEOFF: flying') or st.manager.startswith('LAND: descending')


def ours_down(st, now):
    """Our side is down and disarmed: no tracker armed, no FC armed, nobody in the air, the manager not flying.
    The trackers announce their arm only on a change, so a console started after ARM also reads the FCs, the
    manager's latched status and the heights."""
    if any(a is True for a in st.tracker_armed) or st.arm_complete or manager_flying(st):
        return False
    if not st.twin and any(fc_state(st, i, now) == 'armed' for i in range(st.n)):
        return False
    return not airborne(st, now)


def in_flight(st, now):
    return not ours_down(st, now) or bool(st.his_armed)


def his_link(st):
    """(ok, why): both his nodes subscribe to his command topic and our publisher has matched every subscriber."""
    missing = [n for n in HIS_NODES if n not in st.his_subs]
    if missing:
        return False, 'waiting for ' + ', '.join(m.rsplit('/', 1)[-1] for m in missing)
    if st.his_matched < len(st.his_subs):
        return False, f'matched {st.his_matched} of {len(st.his_subs)} subscribers'
    return True, ''


def partner_armed(st, now):
    """QUAD4 armed: its FC on the rig, his MPC's own flag in the twin."""
    if st.partner is None:
        return False
    if st.twin:
        return bool(st.his_armed)
    return fc_state(st, st.partner, now) == 'armed'


def button_states(st, now, not_ready):
    """{button: (enabled, why not)} for the flight's buttons; DISARM is always enabled."""
    his = st.partner is not None
    link_ok, link_why = his_link(st) if his else (False, 'no partner')
    p_armed = partner_armed(st, now)
    anything_armed = in_flight(st, now) or p_armed
    fleet_armed = st.arm_complete or manager_flying(st)
    manager = '/fleet_manager' in st.nodes

    def rule(*checks):
        for ok, why in checks:
            if not ok:
                return False, why
        return True, ''

    b = {'disarm': (True, '')}
    b['his_arm'] = rule((his, 'no partner'), (not not_ready, 'not READY'), (link_ok, link_why),
                        (not p_armed, 'QUAD4 armed'))
    b['arm'] = rule((not not_ready, 'not READY'), (manager, 'fleet manager not running'),
                    (not st.arm_complete, 'armed'), (not his or p_armed, 'his ARM first'))
    b['takeoff'] = rule((st.arm_complete, f'after "{ARM_DONE}"'),
                        (all(a is not False for a in st.tracker_armed), 'a tracker is not armed'))
    b['his_takeoff'] = rule((his, 'no partner'), (fleet_armed, 'after the fleet ARM'),
                            (p_armed, 'QUAD4 not armed'), (link_ok, link_why))
    b['give_up'] = rule((his, 'no partner'), (st.give_up_subs > 0, 'our planner not listening'),
                        (anything_armed, 'nothing armed'))
    b['land'] = rule((anything_armed, 'nothing armed'))
    b['his_disarm'] = rule((his, 'no partner'), (link_ok, link_why),
                           (ours_down(st, now), 'fleet still armed or up: LAND or DISARM first'))
    return b


def proof_line(st):
    p = st.proof or {}

    def mm(k):
        v = p.get(k)
        return '--' if v is None else f'{1000 * v:.1f} mm'
    text = (f'Magnet: contact {"yes" if st.contact else "no" if st.contact is False else "--"}, '
            f'proof {p.get("state", "--")}')
    if p.get('state') not in (None, 'off', 'watch'):
        text += (f' (try {p.get("tries")}: xy {mm("xy_m")}, normal {mm("normal_m")}, drift {mm("drift_m")}, '
                 f'tether {p.get("body_to_tip_m") or "--"} / free hang {p.get("l_hang_m") or "--"} m)')
    if p.get('fails'):
        text += f', failed {p["fails"]}x (last: {p.get("reason") or "--"})'
    return text + f', attached {"yes" if st.attached else "no" if st.attached is False else "--"}'


class RosLink:
    """The console's node: subscriptions into the state, publishers that live as long as the window."""

    def __init__(self, state, n):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rcl_interfaces.msg import Log
        from std_msgs.msg import Bool, String
        self.Bool, self.String = Bool, String
        rclpy.init()
        self.rclpy = rclpy
        node = self.node = rclpy.create_node('rig_console')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        now = time.time
        for i in range(n):
            ns = f'/drone_{i}'
            node.create_subscription(Bool, f'{ns}/arming_state_feedback',
                                     lambda m, i=i: state.on_tracker_armed(i, m.data), 10)
            node.create_subscription(String, f'{ns}/radio_health', lambda m, i=i: state.on_health(i, m.data, now()), 5)
            node.create_subscription(String, f'{ns}/mux_state', lambda m, i=i: state.on_mux(i, m.data), latched)
        node.create_subscription(Bool, '/tejen/drone_arming_state_feedback', lambda m: state.on_his_armed(m.data), 5)
        node.create_subscription(String, '/fleet/mocap_summary', lambda m: state.on_summary(m.data, now()), 2)
        node.create_subscription(String, '/fleet/manager_status', lambda m: state.on_manager(m.data), latched)
        node.create_subscription(String, '/fleet/phase', lambda m: state.on_phase(m.data), latched)
        node.create_subscription(String, '/join_planner/phase', lambda m: state.on_his_phase(m.data), latched)
        node.create_subscription(String, '/magnet/attach_proof', lambda m: state.on_proof(m.data), 2)
        node.create_subscription(Bool, '/magnet/contact', lambda m: state.on_contact(m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Bool, '/magnet/object_attached', lambda m: state.on_attached(m.data), 5)
        node.create_subscription(Log, '/rosout', lambda m: state.on_log(m.level, m.name, m.msg), 50)
        self.fleet_pub = node.create_publisher(String, FLEET_CMD, 10)
        self.his_pub = node.create_publisher(String, HIS_CMD, 10)
        self.give_up_pub = node.create_publisher(Bool, GIVE_UP, 10)
        self.state = state
        node.create_timer(1.0, self._graph)

    def _graph(self):
        names = {f"{ns.rstrip('/')}/{name}" for name, ns in self.node.get_node_names_and_namespaces()}
        subs = {f"{e.node_namespace.rstrip('/')}/{e.node_name}"
                for e in self.node.get_subscriptions_info_by_topic(HIS_CMD)}
        self.state.on_graph(names, subs, self.his_pub.get_subscription_count(),
                            self.node.count_subscribers(GIVE_UP))

    def spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:
            pass

    def shutdown(self):
        """Stop spinning before the interpreter exits (a node destroyed under a spinning thread aborts)."""
        try:
            self.rclpy.shutdown()
        except Exception:
            pass

    def fleet(self, cmd):
        self.fleet_pub.publish(self.String(data=cmd))

    def his(self, cmd):
        self.his_pub.publish(self.String(data=cmd))

    def give_up(self):
        self.give_up_pub.publish(self.Bool(data=True))


BUTTONS = [('his_arm', 'his ARM', '#7048e8'), ('arm', 'ARM', '#2f9e44'), ('takeoff', 'TAKEOFF', '#1c7ed6'),
           ('his_takeoff', 'his TAKEOFF', '#7048e8'), ('give_up', 'GIVE UP', '#495057'),
           ('land', 'LAND', '#e8590c'), ('disarm', 'DISARM  (space)', '#c92a2a'), ('his_disarm', 'his DISARM', '#7048e8')]


def gui(state, send, exit_after_ms=None, on_ready=None):
    from PyQt5 import QtCore, QtGui, QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    w = QtWidgets.QWidget()
    w.setWindowTitle('Rig console')
    lay = QtWidgets.QVBoxLayout(w)
    ready, air, status, phases, attach = (QtWidgets.QLabel() for _ in range(5))
    for lab in (ready, air, status, phases, attach):
        lab.setWordWrap(True)
        lab.setStyleSheet('font-size: 14px; padding: 4px;')
        lay.addWidget(lab)
    cols = ['drone', 'tracker', 'FC', 'FC / mocap tilt', 'mocap', 'z (m)', 'pack (V)', 'radio', 'mux']
    table = QtWidgets.QTableWidget(state.n, len(cols))
    table.setHorizontalHeaderLabels(cols)
    table.verticalHeader().setVisible(False)
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.setFocusPolicy(QtCore.Qt.NoFocus)
    table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
    table.setFixedHeight(32 + 30 * state.n)
    lay.addWidget(table)
    grid = QtWidgets.QGridLayout()
    btns = {}
    for k, (key, label, colour) in enumerate(BUTTONS):
        b = QtWidgets.QPushButton(label)
        b.setFocusPolicy(QtCore.Qt.NoFocus)     # SPACE must never press a focused button
        b.setMinimumHeight(64)
        b.setStyleSheet(f'QPushButton {{ background: {colour}; color: white; font-weight: bold; font-size: 14px; }}'
                        'QPushButton:disabled { background: #e9ecef; color: #868e96; font-weight: normal; }')
        grid.addWidget(b, k // 4, k % 4)
        btns[key] = (b, label)
    lay.addLayout(grid)
    action = QtWidgets.QLabel('')
    action.setStyleSheet('font-size: 13px; padding: 4px;')
    lay.addWidget(action)
    lay.addWidget(QtWidgets.QLabel('Warnings and errors (newest first) and what they mean:'))
    alerts = QtWidgets.QTableWidget(0, 4)
    alerts.setHorizontalHeaderLabels(['time', 'node', 'message', 'what it means'])
    alerts.verticalHeader().setVisible(False)
    alerts.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    alerts.setFocusPolicy(QtCore.Qt.NoFocus)
    alerts.setWordWrap(True)
    h = alerts.horizontalHeader()
    for c, mode in enumerate(('ResizeToContents', 'ResizeToContents', 'Stretch', 'Stretch')):
        h.setSectionResizeMode(c, getattr(QtWidgets.QHeaderView, mode))
    lay.addWidget(alerts, 1)
    red, amber, green, grey = (QtGui.QColor(c) for c in ('#ffc9c9', '#ffec99', '#d3f9d8', '#f1f3f5'))

    def cell(r, c, text, bg=None, tbl=table):
        it = QtWidgets.QTableWidgetItem(text)
        if bg is not None:
            it.setBackground(bg)
        tbl.setItem(r, c, it)

    def note(text, alarm=False):
        action.setText(f'{time.strftime("%H:%M:%S")}  {text}')
        action.setStyleSheet('font-size: 13px; padding: 4px;' + (' background: #e03131; color: white;' if alarm else ''))

    disarm_left = {'n': 0}
    disarm_timer = QtCore.QTimer()
    disarm_timer.setInterval(DISARM_PERIOD_MS)

    def disarm():
        send.fleet('DISARM')                   # never blocked
        disarm_left['n'] = DISARM_REPEATS - 1
        disarm_timer.start()
        st = state.snapshot()
        up = airborne(st, time.time())
        note('sent DISARM' + (' with ' + ', '.join(f'{quad(i)} {z:.2f} m up' for i, z in up) if up else ''),
             alarm=bool(up))

    def disarm_repeat():
        if disarm_left['n'] <= 0:
            disarm_timer.stop()
            return
        send.fleet('DISARM')
        disarm_left['n'] -= 1

    disarm_timer.timeout.connect(disarm_repeat)
    space = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), w)
    space.setContext(QtCore.Qt.ApplicationShortcut)
    space.activated.connect(disarm)

    def press(key):
        if key == 'disarm':
            disarm()
            return
        st, now = state.snapshot(), time.time()
        ok, why = button_states(st, now, readiness(st, now))[key]
        if not ok:                             # the state moved since the last refresh
            note(f'{dict((k, lab) for k, (_, lab) in btns.items())[key]} not sent: {why}', alarm=True)
            return
        if key == 'arm':
            send.fleet('ARM')
        elif key == 'takeoff':
            send.fleet('TAKEOFF')
        elif key == 'land':
            send.fleet('LAND')
        elif key == 'give_up':
            send.give_up()
        else:
            send.his({'his_arm': 'ARM', 'his_takeoff': 'TAKEOFF', 'his_disarm': 'DISARM'}[key])
            if key == 'his_takeoff':
                ack['t'] = now
        note(f'sent {btns[key][1]}')

    for key, (b, _) in btns.items():
        b.clicked.connect(lambda _=False, key=key: press(key))

    shown = {'n': -1}
    ack = {'t': None}         # his TAKEOFF sent: his planner must leave WAIT_FOR_TAKEOFF within HIS_TAKEOFF_ACK_S

    def refresh():
        now = time.time()
        st = state.snapshot()
        if ack['t'] is not None and st.his_phase not in ('', 'WAIT_FOR_TAKEOFF'):
            ack['t'] = None
        elif ack['t'] is not None and now - ack['t'] > HIS_TAKEOFF_ACK_S:
            ack['t'] = None
            note(f'his planner did not enter TAKEOFF within {HIS_TAKEOFF_ACK_S:.0f} s of his TAKEOFF (still '
                 f'{st.his_phase or "--"}): his MPC may lift alone. LAND if QUAD4 leaves its mat', alarm=True)
        why = readiness(st, now)
        if in_flight(st, now):
            ready.setText('IN FLIGHT (armed)')
            ready.setStyleSheet('font-size: 18px; padding: 6px; font-weight: bold; background: #ffe066;')
        elif not why:
            ready.setText('READY to arm')
            ready.setStyleSheet('font-size: 18px; padding: 6px; font-weight: bold; background: #2f9e44; color: white;')
        else:
            ready.setText('NOT READY: ' + '; '.join(why[:5]) + (f' (+{len(why) - 5} more)' if len(why) > 5 else ''))
            ready.setStyleSheet('font-size: 15px; padding: 6px; font-weight: bold; background: #e03131; color: white;')
        up = airborne(st, now)
        air.setText('In the air: ' + ', '.join(f'{quad(i)} {z:.2f} m' for i, z in up) if up else 'Nobody in the air')
        air.setStyleSheet('font-size: 14px; padding: 4px;' + (' background: #ffec99;' if up else ''))
        bad = any(k in st.manager for k in ALARM_WORDS)
        status.setText(f'Fleet manager: {st.manager or "--"}')
        status.setStyleSheet('font-size: 14px; padding: 4px; font-weight: bold; '
                             + ('background: #e03131; color: white;' if bad else 'background: #f1f3f5;'))
        phases.setText(f'Our phase: {st.phase or "--"}    his phase: {st.his_phase or "--"}')
        bs = bodies(st, now) or {}
        ring = bs.get('payload') or {}
        attach.setText(proof_line(st) + ('' if ring.get('z') is None else
                                         f'    ring z {ring["z"]:.2f} m, tilt {ring.get("tilt_deg") or 0:.1f} deg'))
        for i in range(st.n):
            cell(i, 0, quad(i) + (' (his)' if i == st.partner else ''))
            a = st.tracker_armed[i]
            cell(i, 1, '--' if a is None else 'ARMED' if a else 'disarmed', amber if a else None)
            hh = st.health[i] or {}
            fs = None if st.twin else fc_state(st, i, now)
            if st.twin:
                cell(i, 2, '(twin)', grey)
            else:
                ok, text = fc_check(st, i, now)
                cell(i, 2, text if fs != 'armed' else f'{hh.get("mode")} (armed)',
                     amber if fs == 'armed' else green if ok else red)
            b = bs.get(f'drone_{i}') or {}
            ft, mt = hh.get('fc_tilt_deg'), b.get('tilt_deg')
            off = ft is not None and mt is not None and abs(ft - mt) > FC_TILT_OFF_DEG
            cell(i, 3, f'{"--" if ft is None else f"{ft:.0f}"} / {"--" if mt is None else f"{mt:.0f}"} deg',
                 amber if off else None)
            age = b.get('age_s')
            cell(i, 4, 'none' if age is None else f'{b.get("rate_hz") or 0:.0f} Hz, {1000 * age:.0f} ms',
                 red if age is None or age > POSE_STALE_S else green)
            rest = st.rest_z.get(f'drone_{i}')
            z = b.get('z')
            cell(i, 5, '--' if z is None else f'{z:.2f}' + (f'  (+{z - rest:.2f})' if rest is not None
                                                           and z - rest > AIRBORNE_M else ''))
            if st.twin:
                cell(i, 6, '--', grey)
                cell(i, 7, '--', grey)
            else:
                ok, text = pack_check(st, i, now)
                cell(i, 6, text, green if ok else red)
                radio = ('--' if not hh else 'port closed' if not hh.get('connected') else
                         f'{hh.get("cmd_rate_hz") or 0:.0f} Hz, lq {hh.get("lq") if hh.get("lq") is not None else "--"}'
                         + (f', {hh["reconnects"]} reopen' if hh.get('reconnects') else ''))
                cell(i, 7, radio, red if hh and not hh.get('connected') else None)
            cell(i, 8, st.mux[i] or '--', red if st.mux[i] == 'latched' else grey)
        for key, (ok, why_not) in button_states(st, now, why).items():
            b, label = btns[key]
            b.setEnabled(ok)
            b.setText(label if ok or not why_not else f'{label}\n({why_not})')
        if len(st.alerts) != shown['n']:
            shown['n'] = len(st.alerts)
            alerts.setRowCount(len(st.alerts))
            for r, (t, lvl, node, text, expl) in enumerate(st.alerts):
                bg = red if lvl == 'ERROR' else amber
                cell(r, 0, time.strftime('%H:%M:%S', time.localtime(t)), bg, alerts)
                cell(r, 1, node, bg, alerts)
                cell(r, 2, text, bg, alerts)
                cell(r, 3, expl or '(no explanation on file)', None, alerts)
            alerts.resizeRowsToContents()

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(200)
    w.resize(1250, 820)
    w.show()
    refresh()
    if on_ready:                               # tests: drive the window once it is up
        QtCore.QTimer.singleShot(0, lambda: on_ready(w, btns, space))
    if exit_after_ms:
        QtCore.QTimer.singleShot(exit_after_ms, app.quit)
    return app.exec_()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--drones', type=int, default=4)
    ap.add_argument('--partner', type=int, default=3, help="the drone Tejen's stack flies (default 3 = QUAD4)")
    ap.add_argument('--no-partner', action='store_true', help='our fleet alone')
    ap.add_argument('--twin', action='store_true', help='the twin: no radios, so no FC or pack checks')
    ap.add_argument('--small-angle', type=float, default=25.0,
                    help="the FCs' small_angle (Betaflight default 25; 180 once docs/rig/fc_request.md is done)")
    a = ap.parse_args(argv)
    state = ConsoleState(a.drones, None if a.no_partner else a.partner, a.twin, a.small_angle)
    link = RosLink(state, a.drones)
    spinner = threading.Thread(target=link.spin, daemon=True)
    spinner.start()
    try:
        return gui(state, link)
    finally:
        link.shutdown()
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    sys.exit(main())
