#!/usr/bin/env python3
"""
tools/bench_latency.py -- split the command and mocap delays on the bench, props OFF, no ROS.

One process, one clock: it drives the drone's ELRS TX directly (disarmed, throttle low, every aux low, a random
step pattern on ROLL), polls the FC over USB with MSP (RC channels and raw IMU) and reads the mocap UDP stream
itself. Then:
    radio delay = our ROLL step out of the TX serial -> the ROLL the FC reports over USB
    mocap delay = the FC gyro (over USB) -> the mocap body's angular speed, while you twist the drone by hand
Both are net of half the MSP round trip. Neither includes our ROS nodes (motion_capture_publisher, elrs_interface):
if both come out small, the extra delay is in those.

    tools/bench_latency.py --tx /dev/QUAD1 --fc /dev/ttyACM0 --body 11 --seconds 40
    tools/bench_latency.py --tx none ...       # the mocap half only (no radio plugged in)

Before: the rig stack DOWN (this binds UDP 1511 and opens the TX), the Betaflight web app disconnected (it holds the
FC's USB port), props OFF, battery in, USB in. During: twist and tilt the drone by hand, fast, inside the cameras'
view, the whole time.
"""
import argparse, os, socket, struct, sys, threading, time
import numpy as np
import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'drone_communication'))
from drone_communication.elrs_interface import channelsCrsfToChannelsPacket  # noqa: E402

IDLE, RANGE = 993, 820           # as elrs_interface: CRSF centre and half range
LOW = IDLE - RANGE


def msp_frame(cmd):
    return b'$M<' + bytes([0, cmd, cmd])          # no payload: checksum = 0 ^ cmd


class Recorder:
    def __init__(self, a):
        self.a = a
        self.stop = threading.Event()
        self.sent = []        # (t, roll value -1..1)
        self.rc = []          # (t_request, t_reply, roll us)
        self.imu = []         # (t_request, t_reply, gx, gy, gz, ax, ay, az)
        self.mocap = []       # (t, x, y, z, qx, qy, qz, qw)

    def tx(self):
        ser = serial.Serial(self.a.tx, 921600, timeout=0.01)
        rng = np.random.default_rng(1)
        pkt = np.full(16, LOW, dtype=np.uint16)
        pkt[0], pkt[1], pkt[3] = IDLE, IDLE, IDLE   # roll, pitch, yaw centred; pkt[2] throttle and pkt[4:] aux low
        roll, t_next = 0.0, 0.0
        while not self.stop.is_set():
            t = time.perf_counter()
            if t >= t_next:
                roll = float(rng.uniform(-0.4, 0.4))
                t_next = t + float(rng.uniform(0.08, 0.25))
                pkt[0] = IDLE + int(roll * RANGE)
            ser.write(channelsCrsfToChannelsPacket(pkt))
            self.sent.append((time.perf_counter(), roll))
            if ser.in_waiting:
                ser.read(ser.in_waiting)                  # telemetry, unused
            time.sleep(max(0.0, 0.01 - (time.perf_counter() - t)))
        ser.close()

    def msp(self):
        ser = serial.Serial(self.a.fc, 115200, timeout=0.002)
        time.sleep(0.3)
        ser.reset_input_buffer()
        k = 0
        while not self.stop.is_set():
            cmd = 105 if k % 2 == 0 else 102              # MSP_RC, MSP_RAW_IMU
            k += 1
            t0 = time.perf_counter()
            ser.write(msp_frame(cmd))
            buf = b''
            while time.perf_counter() - t0 < 0.05:
                buf += ser.read(ser.in_waiting or 1)
                i = buf.find(b'$M>')
                if i >= 0 and len(buf) >= i + 5 and len(buf) >= i + 6 + buf[i + 3]:
                    size, got = buf[i + 3], buf[i + 4]
                    data = buf[i + 5:i + 5 + size]
                    t1 = time.perf_counter()
                    if got == 105 and size >= 2:
                        self.rc.append((t0, t1, struct.unpack('<H', data[:2])[0]))
                    elif got == 102 and size >= 18:
                        v = struct.unpack('<9h', data[:18])
                        self.imu.append((t0, t1, v[3], v[4], v[5], v[0], v[1], v[2]))
                    break
        ser.close()

    def udp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('0.0.0.0', 1511))
        s.settimeout(0.2)
        tag = f'{self.a.body}|'.encode()
        while not self.stop.is_set():
            try:
                d, _ = s.recvfrom(512)
            except socket.timeout:
                continue
            t = time.perf_counter()
            if not d.startswith(tag):
                continue
            try:
                txt = d.decode().replace('(', '').replace(')', '')
                _, p, q = txt.split('|')[:3]
                self.mocap.append((t, *map(float, p.split(',')), *map(float, q.split(','))))
            except ValueError:
                continue
        s.close()


def lag(x_t, x, y_t, y, lags, fs=1000.0, band=(0.5, 15.0)):
    """lag (s) maximizing corr(x(t), y(t + lag)): how long y follows x."""
    from scipy.signal import butter, filtfilt
    t0, t1 = max(x_t[0], y_t[0]) + 1.0, min(x_t[-1], y_t[-1]) - 1.0
    g = np.arange(t0, t1, 1 / fs)
    b, a = butter(2, band, btype='band', fs=fs)
    xs = filtfilt(b, a, np.interp(g, x_t, x))
    cc = []
    for L in lags:
        ys = filtfilt(b, a, np.interp(g + L, y_t, y))
        cc.append(np.corrcoef(xs, ys)[0, 1])
    i = int(np.nanargmax(cc))
    return lags[i], cc[i]


def analyse(r, out):
    sent = np.array(r.sent); rc = np.array(r.rc); imu = np.array(r.imu); mc = np.array(r.mocap)
    np.savez(out, sent=sent, rc=rc, imu=imu, mocap=mc)
    print(f'\nsaved {out}: {len(sent)} TX frames, {len(rc)} MSP RC, {len(imu)} MSP IMU, {len(mc)} mocap samples')
    if len(rc) < 50 or len(imu) < 50:
        sys.exit('too few MSP replies: is the FC on USB, out of CLI mode, the web app disconnected?')
    half = np.median(np.r_[rc[:, 1] - rc[:, 0], imu[:, 1] - imu[:, 0]]) / 2
    print(f'MSP round trip median {2 * half * 1e3:.1f} ms; MSP rate {len(rc) / (rc[-1, 1] - rc[0, 1]):.0f} Hz per stream')
    if len(sent):
        tr = (rc[:, 0] + rc[:, 1]) / 2                 # the FC answered about mid round trip
        d_r, c_r = lag(sent[:, 0], sent[:, 1], tr, rc[:, 2], np.arange(0.0, 0.40, 0.0005), band=(0.2, 20.0))
        print(f'RADIO  our TX write -> the FC has it : {d_r * 1e3:6.1f} ms   (corr {c_r:.2f})')
    else:
        print('RADIO  not measured (--tx none)')
    if len(mc) < 200:
        print(f'MOCAP  too few samples of body {r.a.body} ({len(mc)}): is it streamed and in view?')
        return
    ti = (imu[:, 0] + imu[:, 1]) / 2
    w_fc = np.radians(np.linalg.norm(imu[:, 2:5], axis=1))
    tm = mc[:, 0]; q = mc[:, 4:8]
    from scipy.spatial.transform import Rotation as R
    rot = R.from_quat(q)
    dt = np.diff(tm)
    ok = dt > 1e-4
    rel = rot[1:][ok] * rot[:-1][ok].inv()
    w_mc = np.linalg.norm(rel.as_rotvec(), axis=1) / dt[ok]
    tw = (tm[1:][ok] + tm[:-1][ok]) / 2
    print(f'mocap body {r.a.body}: {len(mc) / (tm[-1] - tm[0]):.0f} Hz; angular speed peak FC {w_fc.max():.1f}, '
          f'mocap {np.percentile(w_mc, 99):.1f} rad/s')
    d_m, c_m = lag(ti, w_fc, tw, w_mc, np.arange(-0.05, 0.40, 0.0005), band=(0.3, 8.0))
    print(f'MOCAP  the drone turns -> we get the pose : {d_m * 1e3:6.1f} ms   (corr {c_m:.2f})')
    print('(15 Oct in flight: radio + mocap ~4 ms; 30 Oct h4n: 95 ms)')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--tx', default='/dev/QUAD1')
    p.add_argument('--fc', default='/dev/ttyACM0')
    p.add_argument('--body', type=int, default=11)
    p.add_argument('--seconds', type=float, default=40.0)
    p.add_argument('--out', default='')
    a = p.parse_args()
    r = Recorder(a)
    jobs = (r.msp, r.udp) if a.tx.lower() in ('', 'none') else (r.tx, r.msp, r.udp)
    th = [threading.Thread(target=f, daemon=True) for f in jobs]
    for t in th:
        t.start()
    print(f'recording {a.seconds:.0f} s: TX {a.tx} (disarmed, throttle low), FC {a.fc}, mocap body {a.body}. '
          f'Twist and tilt the drone by hand now.')
    try:
        t_end = time.perf_counter() + a.seconds
        while time.perf_counter() < t_end:
            time.sleep(1.0)
            print(f'  {a.seconds - (t_end - time.perf_counter()):4.0f} s  TX {len(r.sent)}  RC {len(r.rc)}  '
                  f'IMU {len(r.imu)}  mocap {len(r.mocap)}', flush=True)
    except KeyboardInterrupt:
        pass
    r.stop.set()
    for t in th:
        t.join(timeout=1.0)
    out = a.out or os.path.join('results', 'rig', time.strftime('%Y-%m-%d'), time.strftime('bench_latency_%H%M%S.npz'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    analyse(r, out)


if __name__ == '__main__':
    main()
