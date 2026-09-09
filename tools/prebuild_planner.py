#!/usr/bin/env python3
"""
tools/prebuild_planner.py — compile the planner OCP for each fleet size, once.

    ./tools/prebuild_planner.py            # n = 2, 3, 4
    ./tools/prebuild_planner.py 3          # just one

WHY THIS EXISTS. The acados build takes 9-40 s and runs inside the planner node's
__init__. Normally the cached .so is reused and nobody notices. But after any change to
the model sources the first launch must rebuild -- and a harness that starts its
scenario clock on "poses are flowing" will run the whole run and SIGINT the node while
`make` is still going. The interrupted build deletes the old .so and never writes the
signature, so the NEXT run rebuilds too, and so on: the bench never succeeds again and
the failure looks like "the planner publishes no references" rather than "it is still
compiling". That cost a full gate cycle to spot.

It also builds the DYNAMICS EXACTLY AS planner_node DOES. The cache signature covers
load mass, inertia and drone mass, so a prebuild done with guessed constants writes a
signature the node will not match, rebuilds anyway, and looks like it did nothing.

Run this after touching load_cable_dynamics.py / planner_ocp.py, and before benching.
"""
import argparse
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'controller_load_mpc'))

from controller_load_mpc.geometry import attach_points          # noqa: E402
from controller_load_mpc.load_cable_dynamics import LoadCableDynamics  # noqa: E402
from controller_load_mpc.params import (ATTACH_RADIUS, ATTACH_Z,  # noqa: E402
                                        CABLE_LEN, LOAD_MASS)
from controller_load_mpc.planner_node import DRONE_MASS, LOAD_INERTIA  # noqa: E402
from controller_load_mpc.planner_solver import PlannerSolver    # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('n', nargs='*', type=int, default=[2, 3, 4])
    ap.add_argument('--load-mass', type=float, default=LOAD_MASS,
                    help='must match the launch, because the cache signature covers '
                         'it (geometry no longer does)')
    args = ap.parse_args()
    print(f'  load_mass={args.load_mass}  (attachment geometry is a RUNTIME parameter '
          f'now, so it does not affect the cache)')
    for n in args.n:
        rho = attach_points(n, ATTACH_RADIUS, ATTACH_Z)
        dyn = LoadCableDynamics(n, args.load_mass, LOAD_INERTIA,
                                [CABLE_LEN] * n, rho, DRONE_MASS)
        t0 = time.time()
        ps = PlannerSolver(dyn)
        print(f'  n={n}: ready in {time.time() - t0:5.1f} s   '
              f'(p = 4 q_ref + {len(ps._geom)} geometry)')
    print('\n  Geometry (rho, cable length) is a RUNTIME parameter, so these solvers '
          'are\n  valid for any attachment layout at this fleet size.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
