#!/usr/bin/env python3
"""
tools/fc_probe.py -- ask a Betaflight flight controller over USB what it sees and how it
is configured, without the Configurator. Plug the drone's FC into USB (/dev/ttyACM0),
props OFF, radio link up.

    tools/fc_probe.py rc            # stream the 16 RC channels the FC receives; press the
                                    # panel MAGNET toggle and see which channel jumps
    tools/fc_probe.py cli           # dump the mode (aux) table, PINIO setup and receiver
                                    # protocol from the CLI, decoded

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


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ('rc', 'cli'):
        sys.exit(__doc__)
    (rc_watch if sys.argv[1] == 'rc' else cli_dump)(port_from_args(sys.argv[2:]))
