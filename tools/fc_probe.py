#!/usr/bin/env python3
"""
tools/fc_probe.py -- ask a Betaflight flight controller over USB what it sees and how it
is configured, without the Configurator. Plug the drone's FC into USB (/dev/ttyACM0),
props OFF, radio link up.

    tools/fc_probe.py rc            # stream the 16 RC channels the FC receives; press the
                                    # panel MAGNET toggle and see which channel jumps
    tools/fc_probe.py cli           # dump the mode (aux) table, PINIO setup and receiver
                                    # protocol from the CLI, decoded
    tools/fc_probe.py armcheck --tx=/dev/QUAD1 --seconds=60
                                    # read-only, once a second: what blocks arming, the attitude
                                    # the FC believes, its raw accelerometer tilt, the gyro at rest;
                                    # --tx holds the radio link (disarmed) so RXLOSS hides nothing

Channel naming: Betaflight counts CRSF channels 1..16 as ROLL PITCH THR YAW AUX1..AUX12.
ELRSCommand.channel_k maps to packet slot k (k<=3) or k+1 (k>=4), i.e. CRSF channel k+1
or k+2, so channel_10 -> CRSF 12 -> AUX8.

`cli` leaves the FC in CLI mode: power-cycle it (or run `exit` in the Configurator CLI)
before flying or before running `rc` again.
"""
import struct
import sys
import time

import serial

BAUD = 115200
NAMES = ['ROLL', 'PITCH', 'THR', 'YAW'] + [f'AUX{i}' for i in range(1, 13)]
BOX_NAMES = {0: 'ARM', 1: 'ANGLE', 2: 'HORIZON', 13: 'BEEPER', 26: 'FLIP', 27: 'AIRMODE',
             35: 'PREARM', 40: 'USER1', 41: 'USER2', 42: 'USER3', 43: 'USER4'}


def port_from_args(args):
    for a in args:
        if a.startswith('/dev/'):
            return a
    return '/dev/ttyACM0'


# ── MSP v1 ─────────────────────────────────────────────────────────────────────
def msp_request(ser, cmd, payload=b''):
    frame = bytes([len(payload), cmd]) + payload
    chk = 0
    for b in frame:
        chk ^= b
    ser.write(b'$M<' + frame + bytes([chk]))
    deadline = time.time() + 1.0
    buf = b''
    while time.time() < deadline:
        buf += ser.read(64)
        i = buf.find(b'$M>')
        if i >= 0 and len(buf) >= i + 5:
            size = buf[i + 3]
            if len(buf) >= i + 5 + size + 1:
                return buf[i + 5:i + 5 + size]
    return None


def rc_watch(port):
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.5)
    ver = msp_request(ser, 1)                      # MSP_API_VERSION
    if ver is None:
        sys.exit('no MSP reply: is the FC in CLI mode (power-cycle it) or the port wrong?')
    print(f'MSP API {ver[1]}.{ver[2]} on {port}. Streaming RC channels; Ctrl-C to stop.')
    print('Press the panel MAGNET toggle: the channel that jumps is the one on the air.\n')
    last = None
    t0 = time.time()
    while True:
        data = msp_request(ser, 105)               # MSP_RC
        if data is None:
            continue
        n = len(data) // 2
        ch = list(struct.unpack('<' + 'H' * n, data[:2 * n]))
        changed = [k for k in range(n) if last is not None and abs(ch[k] - last[k]) > 50]
        if last is None or changed or int(time.time() - t0) % 2 == 0:
            line = '  '.join(f'{NAMES[k] if k < len(NAMES) else k}={ch[k]}' for k in range(n))
            flag = ('   <-- CHANGED: ' + ', '.join(NAMES[k] for k in changed)) if changed else ''
            print(f'{time.time() - t0:6.1f}s  {line}{flag}')
        last = ch
        time.sleep(0.2)


# ── CLI ────────────────────────────────────────────────────────────────────────
def cli_read(ser, quiet_s=0.6, max_s=8.0):
    buf = b''
    t_last = time.time()
    t0 = t_last
    while True:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            t_last = time.time()
        elif time.time() - t_last > quiet_s or time.time() - t0 > max_s:
            break
    return buf.decode(errors='replace')


def cli_dump(port):
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.5)
    ser.write(b'#')
    banner = cli_read(ser)
    if 'CLI' not in banner and '#' not in banner:
        sys.exit(f'no CLI prompt from {port}; got: {banner[:80]!r}')
    out = {}
    for cmd in ('version', 'aux', 'resource', 'get pinio_config', 'get pinio_box',
                'get serialrx_provider', 'get rx_spi_protocol'):
        ser.write((cmd + '\n').encode())
        out[cmd] = cli_read(ser)
    print(out['version'].strip())
    print('\n=== modes (aux <slot> <box> <auxChannelIndex> <lo> <hi> ...): '
          'auxChannelIndex 0 = AUX1, 7 = AUX8; a range like 1700-2100 = ON when high')
    for line in out['aux'].splitlines():
        if line.startswith('aux ') and not line.rstrip().endswith('900 900 0 0'):
            f = line.split()
            box = int(f[2]); aux = int(f[3])
            print(f'  {line.strip():40s}  -> {BOX_NAMES.get(box, "box" + str(box))} on AUX{aux + 1}, '
                  f'range {f[4]}-{f[5]}')
    print('\n=== PINIO (the pins USER modes drive):')
    for line in out['resource'].splitlines():
        if 'PINIO' in line:
            print('  ' + line.strip())
    for k in ('get pinio_config', 'get pinio_box'):
        v = [l for l in out[k].splitlines() if '=' in l]
        print('  ' + (v[0].strip() if v else f'{k}: (not available on this build)'))
    print('\n=== receiver:')
    for k in ('get serialrx_provider', 'get rx_spi_protocol'):
        v = [l for l in out[k].splitlines() if '=' in l]
        if v:
            print('  ' + v[0].strip())
    print('\nFC is now in CLI mode: power-cycle it before flying or before `rc`.')


# ── arm check (read-only) ──────────────────────────────────────────────────────
# Betaflight 4.3 armingDisableFlags, bit order
ARM_FLAGS = ['NOGYRO', 'FAILSAFE', 'RXLOSS', 'BADRX', 'BOXFAILSAFE', 'RUNAWAY', 'CRASH', 'THROTTLE', 'ANGLE',
             'BOOTGRACE', 'NOPREARM', 'LOAD', 'CALIB', 'CLI', 'CMS', 'BST', 'MSP', 'PARALYZE', 'GPS', 'RESCUE_SW',
             'RPMFILTER', 'REBOOT_REQD', 'DSHOT_BBANG', 'NO_ACC_CAL', 'MOTOR_PROTO', 'ARMSWITCH']


def arming_flags(status_ex):
    """Names of the set armingDisableFlags in an MSP_STATUS_EX reply."""
    n = status_ex[15]                              # bytes of extra flight-mode flags after the fixed 16
    i = 16 + n
    count, flags = status_ex[i], struct.unpack('<I', status_ex[i + 1:i + 5])[0]
    return [ARM_FLAGS[b] if b < len(ARM_FLAGS) else f'bit{b}' for b in range(min(count, 32)) if flags >> b & 1]


def tx_hold_disarmed(tx_port, stop):
    """Keep the link up with a disarmed frame (sticks centred, throttle and every aux low) every 10 ms."""
    import os
    import numpy as np
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'drone_communication'))
    from drone_communication.elrs_interface import channelsCrsfToChannelsPacket
    tx = serial.Serial(tx_port, 921600, timeout=0.01)
    pkt = np.full(16, 173, dtype=np.uint16)
    pkt[0] = pkt[1] = pkt[3] = 993
    frame = channelsCrsfToChannelsPacket(pkt)
    while not stop.is_set():
        tx.write(frame)
        if tx.in_waiting:
            tx.read(tx.in_waiting)
        time.sleep(0.01)
    tx.close()


def arm_check(port, tx_port=None, seconds=60.0):
    """Every second: the FC's arming-disable flags, the attitude it believes, the tilt its raw accelerometer gives
    and the gyro at rest. Nothing is written to the FC. With tx_port, our TX holds the link (disarmed) so RXLOSS
    does not hide the rest."""
    import math
    import threading
    stop = threading.Event()
    if tx_port:
        threading.Thread(target=tx_hold_disarmed, args=(tx_port, stop), daemon=True).start()
    ser = serial.Serial(port, BAUD, timeout=0.05)
    time.sleep(0.5)
    if msp_request(ser, 1) is None:
        sys.exit('no MSP reply: is the FC in CLI mode (power-cycle it), the web app connected, or the port wrong?')
    print('   t  FC roll/pitch (deg)  FC tilt   acc tilt  |acc| g   gyro dps (x,y,z)   arming blocked by')
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            st, att, imu = msp_request(ser, 150), msp_request(ser, 108), msp_request(ser, 102)
            if not (st and att and imu):
                continue
            roll, pitch = (v / 10.0 for v in struct.unpack('<2h', att[:4]))
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(roll)) * math.cos(math.radians(pitch))))))
            a = struct.unpack('<9h', imu[:18])
            ax, ay, az = a[0:3]
            norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
            acc_tilt = math.degrees(math.acos(max(-1.0, min(1.0, az / norm))))
            flags = arming_flags(st)
            print(f'{time.time() - t0:4.0f}  {roll:+7.1f} {pitch:+7.1f}      {tilt:5.1f}    {acc_tilt:5.1f}    '
                  f'{norm / 512:4.2f}   {a[3]:+4d} {a[4]:+4d} {a[5]:+4d}     {" ".join(flags) or "nothing (would arm)"}',
                  flush=True)
            time.sleep(max(0.0, 1.0 - (time.time() - t0) % 1.0))
    except KeyboardInterrupt:
        pass
    stop.set()


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ('rc', 'cli', 'armcheck'):
        sys.exit(__doc__)
    if sys.argv[1] == 'armcheck':
        rest = sys.argv[2:]
        tx = next((a.split('=', 1)[1] for a in rest if a.startswith('--tx=')), None)
        secs = float(next((a.split('=', 1)[1] for a in rest if a.startswith('--seconds=')), 60))
        arm_check(port_from_args(rest), tx, secs)
    else:
        (rc_watch if sys.argv[1] == 'rc' else cli_dump)(port_from_args(sys.argv[2:]))
