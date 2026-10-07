#!/usr/bin/env python3
"""tools/gz_push.py -- add a persistent force to the ring in a running Gazebo twin (a *_push world).

    python3 tools/gz_push.py --fz -0.98      # the rig's 100 g drop
    python3 tools/gz_push.py --fz 0.98       # take it off again (persistent forces add up)

`gz topic -p` loses its one message while the connection forms (as the runner did, R0855), so this
waits for Gazebo's ApplyLinkWrench subscriber before publishing.
"""
import argparse
import sys
import time

from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.transport13 import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fx', type=float, default=0.0)
    ap.add_argument('--fy', type=float, default=0.0)
    ap.add_argument('--fz', type=float, default=0.0)
    ap.add_argument('--link', default='lift_system::payload::body')
    ap.add_argument('--world', default='quadcopter')
    a = ap.parse_args()
    node = Node()
    pub = node.advertise(f'/world/{a.world}/wrench/persistent', EntityWrench)
    t0 = time.monotonic()
    while not pub.has_connections():
        if time.monotonic() - t0 > 5.0:
            sys.exit('no subscriber on the wrench topic: is Gazebo running a *_push world?')
        time.sleep(0.05)
    time.sleep(0.2)
    w = EntityWrench()
    w.entity.name = a.link
    w.entity.type = Entity.LINK
    w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = a.fx, a.fy, a.fz
    pub.publish(w)
    time.sleep(0.5)
    print(f'added ({a.fx:+.2f}, {a.fy:+.2f}, {a.fz:+.2f}) N on {a.link}')


if __name__ == '__main__':
    main()
