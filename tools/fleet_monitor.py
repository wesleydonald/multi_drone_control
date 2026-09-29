#!/usr/bin/env python3
"""tools/fleet_monitor.py -- one window with what the operator needs during a rig (or sim) flight.

    python3 tools/fleet_monitor.py --drones 4

Top: the fleet manager's latest decision (red when something was refused or stopped) and the planner phase.
Table: per drone, armed, mocap age, height, tilt, throttle (red when pinned at the 0.6 cap), battery, mux.
Ring: height and tilt. Bottom: warnings and errors from every node (/rosout), newest first, each with a plain
explanation of what it means and what to do. Read-only: it never publishes.
"""
import argparse
import math
import re
import sys
import threading
import time
from collections import deque

# What a known warning or error means, and what to do. First match wins; keep the patterns specific.
EXPLAIN = [
    (r'Pose timeout', 'That drone lost mocap longer than the pose timeout (0.25 s on the rig): markers covered, '
                      'the body lost in Motive, or the mocap stream stalled. Its tracker disarmed it.'),
    (r'disarmed before TAKEOFF', 'A tracker dropped out between ARM and TAKEOFF, so the manager grounded the fleet '
                                 '(the others would lift the ring one-sided). Usually a pose timeout: see the line '
                                 'before it. Relaunch T2 to retry.'),
    (r'TAKEOFF REFUSED', 'Not every drone was armed when TAKEOFF came, so the fleet was disarmed. Relaunch T2, ARM, '
                         'wait for every drone to show ARMED.'),
    (r'ARM REFUSED: mux latched', 'An earlier abort latched that drone\'s mux (it holds the motors off until it '
                                  'restarts). Relaunch the launch that started the muxes.'),
    (r'ARM FAILED|did not answer ARM|refused ARM', 'A tracker was not up (not started, or still building its solver) '
                                                    'when ARM came. Wait for every "Controller ready", relaunch T2.'),
    (r'EMERGENCY STOP', 'The fleet was stopped (the reason is at the end of the line): DISARM/ESTOP pressed, or a drone '
                        'faulted in flight.'),
    (r'disarmed in flight', 'A drone disarmed itself in the air (pose timeout, envelope, stale reference): the manager '
                            'stopped the whole fleet. That drone\'s own line says why.'),
    (r'ENVELOPE|envelope fault', 'A drone left the flight envelope (tilt > 60 deg or speed > 3 m/s) and was disarmed.'),
    (r'reference .* s old|reference stale', 'The planner stopped sending references (crashed, blocked or never '
                                            'started): the tracker stops rather than fly an old plan.'),
    (r'Radio .* not available|No /dev/ttyUSB', 'That drone\'s TX module is not connected: it gets no commands. Reseat '
                                               'the USB, check ls -l /dev/QUAD*, relaunch T1.'),
    (r'trim hit its bound', 'The typed kT is far from this drone\'s real thrust (the trim ran out of range). LAND and '
                            'type the kT the thrust check measured.'),
    (r'arc creep timed out', 'The creep did not reach 45 deg in time: rods not coupled to the ring, drones not pulling '
                             '(thrust cap?), or the rod length is wrong. LAND.'),
    (r'liftoff gate timed out', 'The drones did not rise off the floor at the start of the creep (thrust, or a drone '
                                'still sitting on a rod). Watch the throttle column.'),
    (r'FLEET ABORT', 'The tracker received the fleet abort (see the EMERGENCY STOP line for the reason).'),
    (r'solve failed|acados returned status', 'The MPC solver failed; the tracker holds its last good command. '
                                              'Repeated failures disarm it.'),
    (r'not reachable|not running', 'A node the command needs is not up: check the terminal that should start it.'),
]
CAP = 0.6
ALARM_WORDS = ('REFUSED', 'FAILED', 'EMERGENCY', 'disarmed before', 'disarmed in flight', 'Cannot')


def explain(text):
    for pat, why in EXPLAIN:
        if re.search(pat, text):
            return why
    return ''


def tilt_deg(qw, qx, qy, qz):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))))


class FleetState:
    """Everything the window shows, filled by ROS callbacks, read by the GUI timer (under self.lock)."""

    def __init__(self, n):
        self.n = n
        self.lock = threading.Lock()
        self.manager = ''
        self.phase = ''
        self.drones = [dict(armed=None, pose_t=None, z=math.nan, tilt=math.nan, thr=math.nan, cap_since=None,
                            volt=math.nan, mux='') for _ in range(n)]
        self.ring = dict(z=math.nan, tilt=math.nan, t=None)
        self.alerts = deque(maxlen=200)      # (wall time, level, node, text, why)

    def on_pose(self, i, msg):
        p, q = msg.pose.position, msg.pose.orientation
        with self.lock:
            d = self.drones[i]
            d.update(pose_t=time.time(), z=p.z, tilt=tilt_deg(q.w, q.x, q.y, q.z))

    def on_ring(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.ring.update(z=p.z, tilt=tilt_deg(q.w, q.x, q.y, q.z), t=time.time())

    def on_cmd(self, i, msg):
        thr = 0.5 * (msg.channel_2 + 1.0) if msg.armed else 0.0
        with self.lock:
            d = self.drones[i]
            d['thr'] = thr
            if thr >= CAP - 0.01:
                d['cap_since'] = d['cap_since'] or time.time()
            else:
                d['cap_since'] = None

    def on_armed(self, i, msg):
        with self.lock:
            self.drones[i]['armed'] = bool(msg.data)

    def on_telemetry(self, i, msg):
        with self.lock:
            self.drones[i]['volt'] = float(msg.battery_voltage)

    def on_mux(self, i, msg):
        with self.lock:
            self.drones[i]['mux'] = msg.data

    def on_manager(self, msg):
        with self.lock:
            self.manager = msg.data

    def on_phase(self, msg):
        with self.lock:
            self.phase = msg.data

    def on_log(self, level, node, text):
        if level < 30:                         # rcl_interfaces/Log: 30 WARN, 40 ERROR, 50 FATAL
            return
        with self.lock:
            self.alerts.appendleft((time.time(), 'ERROR' if level >= 40 else 'WARN', node, text, explain(text)))


def ros_thread(state, n):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rcl_interfaces.msg import Log
    from std_msgs.msg import Bool, String
    from interfaces.msg import ELRSCommand, MotionCaptureState, Telemetry
    rclpy.init()
    node = rclpy.create_node('fleet_monitor')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for i in range(n):
        ns = f'/drone_{i}'
        node.create_subscription(MotionCaptureState, f'{ns}/motion_capture_state', lambda m, i=i: state.on_pose(i, m), 5)
        node.create_subscription(ELRSCommand, f'{ns}/ELRSCommand', lambda m, i=i: state.on_cmd(i, m), 5)
        node.create_subscription(Bool, f'{ns}/arming_state_feedback', lambda m, i=i: state.on_armed(i, m), 5)
        node.create_subscription(Telemetry, f'{ns}/telemetry', lambda m, i=i: state.on_telemetry(i, m), 5)
        node.create_subscription(String, f'{ns}/mux_state', lambda m, i=i: state.on_mux(i, m), latched)
    node.create_subscription(MotionCaptureState, '/payload/motion_capture_state', state.on_ring, 5)
    node.create_subscription(String, '/fleet/manager_status', state.on_manager, latched)
    node.create_subscription(String, '/fleet/phase', state.on_phase, latched)
    node.create_subscription(Log, '/rosout', lambda m: state.on_log(m.level, m.name, m.msg), 50)
    try:
        rclpy.spin(node)
    except Exception:
        pass


def gui(state, exit_after_ms=None):
    from PyQt5 import QtCore, QtGui, QtWidgets
    app = QtWidgets.QApplication(sys.argv)
    w = QtWidgets.QWidget()
    w.setWindowTitle('Fleet monitor')
    lay = QtWidgets.QVBoxLayout(w)
    status = QtWidgets.QLabel('Fleet manager: --')
    status.setWordWrap(True)
    phase = QtWidgets.QLabel('Phase: --')
    ring = QtWidgets.QLabel('Ring: --')
    for lab in (status, phase, ring):
        lab.setStyleSheet('font-size: 14px; padding: 4px;')
        lay.addWidget(lab)
    cols = ['drone', 'armed', 'mocap age', 'z (m)', 'tilt (deg)', 'throttle', 'battery (V)', 'mux']
    table = QtWidgets.QTableWidget(state.n, len(cols))
    table.setHorizontalHeaderLabels(cols)
    table.verticalHeader().setVisible(False)
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
    lay.addWidget(table)
    lay.addWidget(QtWidgets.QLabel('Warnings and errors (newest first) and what they mean:'))
    alerts = QtWidgets.QTableWidget(0, 4)
    alerts.setHorizontalHeaderLabels(['time', 'node', 'message', 'what it means'])
    alerts.verticalHeader().setVisible(False)
    alerts.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    alerts.setWordWrap(True)
    h = alerts.horizontalHeader()
    h.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
    h.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
    h.setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
    h.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
    lay.addWidget(alerts, 1)
    red, amber, green, grey = (QtGui.QColor(c) for c in ('#ffc9c9', '#ffec99', '#d3f9d8', '#f1f3f5'))

    def cell(r, c, text, bg=None, table=table):
        it = QtWidgets.QTableWidgetItem(text)
        if bg is not None:
            it.setBackground(bg)
        table.setItem(r, c, it)

    shown = {'n': -1}

    def refresh():
        now = time.time()
        with state.lock:
            man, ph = state.manager, state.phase
            drones = [dict(d) for d in state.drones]
            rg = dict(state.ring)
            al = list(state.alerts)
        bad = any(k in man for k in ALARM_WORDS)
        status.setText(f'Fleet manager: {man or "--"}')
        status.setStyleSheet('font-size: 14px; padding: 4px; font-weight: bold; '
                             + ('background: #e03131; color: white;' if bad else 'background: #f1f3f5;'))
        phase.setText(f'Phase: {ph or "--"}')
        age = (now - rg['t']) if rg['t'] else None
        ring.setText('Ring: --' if age is None or age > 1.0 else f'Ring: z {rg["z"]:.2f} m, tilt {rg["tilt"]:.1f} deg')
        for i, d in enumerate(drones):
            cell(i, 0, f'D{i}')
            cell(i, 1, '--' if d['armed'] is None else ('ARMED' if d['armed'] else 'disarmed'),
                 amber if d['armed'] else None)
            a = (now - d['pose_t']) if d['pose_t'] else None
            cell(i, 2, 'none' if a is None else f'{1000 * a:.0f} ms', red if a is None or a > 0.25 else green)
            cell(i, 3, f'{d["z"]:.2f}' if d['z'] == d['z'] else '--')
            cell(i, 4, f'{d["tilt"]:.1f}' if d['tilt'] == d['tilt'] else '--',
                 red if d['tilt'] == d['tilt'] and d['tilt'] > 35 else None)
            at_cap = d['cap_since'] is not None and now - d['cap_since'] >= 2.0
            cell(i, 5, ('--' if d['thr'] != d['thr'] else f'{d["thr"]:.2f}') + ('  AT CAP' if at_cap else ''),
                 red if at_cap else None)
            v = d['volt']
            cell(i, 6, f'{v:.2f}' if v == v else '--',
                 None if v != v else (green if v >= 24.0 else amber if v >= 22.8 else red))
            cell(i, 7, d['mux'] or '--', grey)
        if len(al) != shown['n']:
            shown['n'] = len(al)
            alerts.setRowCount(len(al))
            for r, (t, lvl, node, text, why) in enumerate(al):
                bg = red if lvl == 'ERROR' else amber
                cell(r, 0, time.strftime('%H:%M:%S', time.localtime(t)), bg, alerts)
                cell(r, 1, node, bg, alerts)
                cell(r, 2, text, bg, alerts)
                cell(r, 3, why or '(no explanation on file)', None, alerts)
            alerts.resizeRowsToContents()

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(200)
    w.resize(1100, 700)
    w.show()
    if exit_after_ms:                          # tests: build, refresh a few times, close
        QtCore.QTimer.singleShot(exit_after_ms, app.quit)
    return app.exec_()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--drones', type=int, default=4)
    a = ap.parse_args(argv)
    state = FleetState(a.drones)
    threading.Thread(target=ros_thread, args=(state, a.drones), daemon=True).start()
    return gui(state)


if __name__ == '__main__':
    sys.exit(main())
