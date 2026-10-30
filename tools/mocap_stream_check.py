#!/usr/bin/env python3
"""
tools/mocap_stream_check.py -- is the Motive forwarder keeping up? Run it with the rig stack DOWN (it binds UDP 1511).

The forwarder on the mocap PC sends ~2,250 packets/s at most. Past that it queues and every pose arrives late: on
30 Oct it streamed 10 bodies at 240 Hz (2,400/s), delivered 225 Hz per body and the poses were 95 ms late
(RIG-1030-bench_mocap); every ring lift heaved. Body 11 alone: 240 Hz, 7.5 ms. So stream only the bodies a flight
uses (at most 8 at 240 Hz) and check here that each one arrives at the full camera rate.

    tools/mocap_stream_check.py                                   # 5 s, camera rate 240 Hz
    tools/mocap_stream_check.py --expect 6,8,11,12,13,14,24       # also flag missing / extra bodies

Exit 0 when every body arrives within 2 % of --rate and the expected set matches, else 1.
"""
import argparse, re, socket, sys, time
from collections import Counter

FORWARDER_MAX_PPS = 2250.0       # measured 30 Oct (10 bodies offered 2,400/s)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--seconds', type=float, default=5.0)
    p.add_argument('--rate', type=float, default=240.0, help="Motive's camera frame rate (Hz)")
    p.add_argument('--expect', default='', help='comma-separated body ids the flight uses')
    p.add_argument('--port', type=int, default=1511)
    a = p.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('0.0.0.0', a.port))
    except OSError as e:
        sys.exit(f'cannot bind UDP {a.port} ({e}): stop the rig stack first (the mocap node holds the port)')
    s.settimeout(1.0)
    cnt, senders = Counter(), set()
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        try:
            d, addr = s.recvfrom(512)
        except socket.timeout:
            continue
        senders.add(addr[0])
        m = re.match(rb'\s*(-?\d+)\|', d)
        cnt[int(m.group(1)) if m else 'bad'] += 1
    T = time.time() - t0
    s.close()

    if not cnt:
        print(f'NO MOCAP: 0 packets on UDP {a.port} in {T:.0f} s. The forwarder on the mocap PC must send to this '
              f'laptop (192.168.0.87:{a.port}); check it is running and Motive is streaming.')
        return 1
    total = sum(cnt.values()) / T
    bodies = sorted(k for k in cnt if k != 'bad')
    ok = True
    print(f'from {", ".join(sorted(senders))}: {len(bodies)} bodies, {total:.0f} packets/s '
          f'(forwarder limit ~{FORWARDER_MAX_PPS:.0f}/s)')
    for b in bodies:
        hz = cnt[b] / T
        low = hz < 0.98 * a.rate
        ok &= not low
        print(f'  body {b:3d}: {hz:6.1f} Hz' + ('   <-- below the camera rate: the forwarder is falling behind' if low else ''))
    if cnt['bad']:
        print(f'  {cnt["bad"]} packets not in the "id|(x, y, z)|(qx, qy, qz, qw)" format')
    if a.expect:
        want = {int(x) for x in a.expect.split(',') if x.strip()}
        missing, extra = sorted(want - set(bodies)), sorted(set(bodies) - want)
        if missing:
            ok = False
            print(f'MISSING (not streamed or not tracked): {missing}')
        if extra:
            ok = False
            print(f'EXTRA (streamed but not used here; untick them in Motive): {extra}')
    if len(bodies) * a.rate > 0.95 * FORWARDER_MAX_PPS:
        ok = False
        print(f'TOO MANY: {len(bodies)} bodies x {a.rate:.0f} Hz = {len(bodies) * a.rate:.0f} packets/s, at or over the '
              f'forwarder limit. Stream at most {int(0.95 * FORWARDER_MAX_PPS // a.rate)} bodies at {a.rate:.0f} Hz.')
    print('OK: the forwarder keeps up' if ok else 'NOT OK: fix the streamed bodies before flying (poses arrive late)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
