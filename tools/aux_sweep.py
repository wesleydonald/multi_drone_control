#!/usr/bin/env python3
"""
tools/aux_sweep.py -- find which aux channel a drone's magnet is on, from the radio side.

Publishes ELRSCommand on /drone_<i>/ELRSCommand (disarmed, throttle low, everything
neutral) and raises ONE aux channel at a time to +1 for `hold` seconds, announcing which.
Watch or hold the magnet: the channel that makes it energise is the one Betaflight's
magnet mode sits on. Then relaunch with `rio ... magnet_channel:=<that channel>`.

    tools/aux_sweep.py --drone 0                 # sweep channel_4 .. channel_10 (AUX2..AUX8)
    tools/aux_sweep.py --drone 0 --channels 10   # hold a single channel high

Needs terminal 1 (`rio`) up WITHOUT magnet_initial:=ON and with the panel MAGNET toggles
untouched (a latched magnet value in the radio node overrides the swept channel), and
terminal 2 NOT running (a tracker would publish on the same topic). Props off. Battery in: the magnet is powered
from the pack, not from USB.

Channel naming: ELRSCommand.channel_k -> CRSF channel k+2 for k >= 4, i.e. channel_4 is
AUX2 ... channel_10 is AUX8 (packet slot 4 is the arm switch, AUX1).
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from interfaces.msg import ELRSCommand

AUX = {k: f'AUX{k - 2}' for k in range(4, 11)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drone', type=int, required=True)
    ap.add_argument('--channels', type=int, nargs='+', default=list(range(4, 11)))
    ap.add_argument('--hold', type=float, default=4.0, help='seconds high per channel')
    ap.add_argument('--gap', type=float, default=2.0, help='seconds all-low between channels')
    a = ap.parse_args()
    for k in a.channels:
        if not 4 <= k <= 10:
            raise SystemExit(f'channel_{k} is not an aux channel (use 4..10)')

    rclpy.init()
    node = Node('aux_sweep')
    pub = node.create_publisher(ELRSCommand, f'/drone_{a.drone}/ELRSCommand', 1)

    def send(active, value, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            m = ELRSCommand()
            m.armed = False
            m.channel_2 = -1.0            # throttle LOW
            for k in range(4, 11):
                setattr(m, f'channel_{k}', value if k == active else -1.0)
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.05)

    print(f'drone {a.drone}: sweeping {[f"channel_{k} ({AUX[k]})" for k in a.channels]}, '
          f'{a.hold:.0f} s high each. Listen for the magnet.')
    send(None, -1.0, a.gap)
    for k in a.channels:
        print(f'  >>> channel_{k} ({AUX[k]}) HIGH', flush=True)
        send(k, +1.0, a.hold)
        print(f'      channel_{k} low', flush=True)
        send(None, -1.0, a.gap)
    print('done: all aux low. The radio node returns to its own latched magnet value.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
