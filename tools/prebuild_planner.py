#!/usr/bin/env python3
"""
tools/prebuild_planner.py — compile the planner OCP for each fleet size, once.

    ./tools/prebuild_planner.py            # n = 2, 3, 4
    ./tools/prebuild_planner.py 3          # just one
    ./tools/prebuild_planner.py --from-config configs/experiments/s8_chain4_R50_L100_it1.yaml [--dry-run]

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

--from-config builds what one experiment config's planner loads: its fleet size and, for a detach-mode flight
that resizes its OCP (reconfig_mode ocp), every size params.prebuild_sizes gives, with that flight's masses, ring
inertia, rod model and pin_cable_rates, resolved as the launch does (the sim profile of its mode, its params_file,
then its launch args). An 8-quad chain builds 2..8 (~12 min cold), which must not happen in a run's ready window.

Run this after touching load_cable_dynamics.py / planner_ocp.py, and before benching.
"""
import argparse
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'mpc_planner'))

from mpc_planner.geometry import attach_points          # noqa: E402
from mpc_planner.load_cable_dynamics import LoadCableDynamics  # noqa: E402
from mpc_planner.params import (ATTACH_RADIUS, ATTACH_Z,  # noqa: E402
                                CABLE_LEN, LOAD_MASS)
from mpc_planner.planner_node import DRONE_MASS  # noqa: E402
from mpc_planner.params import load_inertia, prebuild_sizes  # noqa: E402
from mpc_planner.planner_solver import PlannerSolver    # noqa: E402

DETACH_MODES = ('dissipative', 'attach', 'network')      # the modes whose planner is the detach node


def _num(v, default):
    v = '' if v is None else str(v).strip()
    return float(v) if v else default


def config_builds(path):
    """(sizes, build options) one experiment config's planner loads; ([], None) for a mode without a planner."""
    sys.path.insert(0, os.path.join(REPO, 'tools'))
    sys.path.insert(0, os.path.join(REPO, 'src', 'bringup'))
    from experiment.config import ExperimentConfig
    from bringup.profiles import DEFAULT_MODE, LEGACY_DEFAULT_MODE, profile
    cfg = ExperimentConfig.from_yaml(path)
    la = cfg.control_args()
    legacy = str(la.get('legacy', '')).strip().lower() == 'true'
    mode = str(la.get('mode') or (LEGACY_DEFAULT_MODE if legacy else DEFAULT_MODE))
    if mode == 'free_hover':
        return [], None
    pf = str(la.get('params_file') or '')
    vals = dict(profile('sim', mode, os.path.expanduser(pf) if pf else '',
                        cdir=os.path.join(REPO, 'src', 'bringup', 'config'), legacy=legacy))
    vals.update({k: ('true' if v is True else 'false' if v is False else str(v)) for k, v in la.items()})
    n = int(_num(vals.get('num_drones'), cfg.num_drones))
    load = _num(vals.get('load_mass'), LOAD_MASS)
    ixx, izz = _num(vals.get('load_ixx'), 0.0), _num(vals.get('load_izz'), 0.0)
    opts = dict(load_mass=load, drone_mass=_num(vals.get('drone_mass'), DRONE_MASS),
                inertia=[ixx, ixx, izz] if ixx > 0.0 else load_inertia(load),
                rod_mass=_num(vals.get('rod_mass'), 0.0), rod_com=_num(vals.get('rod_com'), 0.0),
                cable_len=_num(vals.get('cable_len'), CABLE_LEN),
                pin_rates=str(vals.get('pin_cable_rates', '')).strip().lower() == 'true')
    sizes = [n]
    if mode in DETACH_MODES and str(vals.get('reconfig_mode', '')).strip() == 'ocp':
        reserved = int(_num(vals.get('reserved_attach'), 0)) if mode == 'attach' else 0
        sizes = prebuild_sizes(n, reserved, int(_num(vals.get('min_survivors'), 3)))
    return sizes, opts


def build(n, load_mass, drone_mass, inertia, rod_mass, rod_com, cable_len, pin_rates):
    rho = attach_points(n, ATTACH_RADIUS, ATTACH_Z)
    lam = rod_com / cable_len if rod_mass > 0.0 else 0.0
    dyn = LoadCableDynamics(n, load_mass, inertia, [cable_len] * n, rho, drone_mass,
                            rod_mass=rod_mass, rod_lam=lam)
    t0 = time.time()
    ps = PlannerSolver(dyn, pin_rates=pin_rates)
    print(f'  n={n}: ready in {time.time() - t0:5.1f} s   (p = 4 q_ref + {len(ps._geom)} geometry)', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('n', nargs='*', type=int, default=[2, 3, 4])
    ap.add_argument('--load-mass', type=float, default=LOAD_MASS,
                    help='must match the launch, because the cache signature covers '
                         'it (geometry no longer does)')
    ap.add_argument('--drone-mass', type=float, default=DRONE_MASS,
                    help='per-drone mass with pack (sim 0.64); in the cache signature too')
    ap.add_argument('--load-ixx', type=float, default=0.0,
                    help='ring inertia (planner load_ixx; with --load-izz); 0 = the M2A ring per kg at --load-mass')
    ap.add_argument('--load-izz', type=float, default=0.0)
    ap.add_argument('--rod-mass', type=float, default=0.0,
                    help='massive-rod model (option F, planner rod_mass); 0 = massless cables')
    ap.add_argument('--rod-com', type=float, default=0.0,
                    help='rod centre of mass from the load end, m (planner rod_com)')
    ap.add_argument('--pin-rates', action='store_true',
                    help='the variant whose node 0 also pins the rod rates (planner pin_cable_rates)')
    ap.add_argument('--cable-len', type=float, default=CABLE_LEN,
                    help='rod length the planner divides rod_com by (must match the launch)')
    ap.add_argument('--from-config', metavar='CFG',
                    help="an experiment config: build its planner's sizes and model (the options above are ignored)")
    ap.add_argument('--dry-run', action='store_true', help='print what would be built and stop')
    args = ap.parse_args()
    if args.from_config:
        sizes, opts = config_builds(args.from_config)
        if opts is None:
            print(f'  {args.from_config}: no planner in this mode, nothing to build')
            return 0
    else:
        if (args.load_ixx > 0.0) != (args.load_izz > 0.0):
            ap.error('--load-ixx and --load-izz go together')
        sizes = args.n
        opts = dict(load_mass=args.load_mass, drone_mass=args.drone_mass,
                    inertia=([args.load_ixx, args.load_ixx, args.load_izz] if args.load_ixx > 0.0
                             else load_inertia(args.load_mass)),
                    rod_mass=args.rod_mass, rod_com=args.rod_com, cable_len=args.cable_len,
                    pin_rates=args.pin_rates)
    print(f'  sizes {sizes}  load_mass={opts["load_mass"]}  drone_mass={opts["drone_mass"]}  '
          f'inertia={[round(v, 6) for v in opts["inertia"]]}  rod_mass={opts["rod_mass"]}  '
          f'pin_rates={opts["pin_rates"]}  (attachment geometry is a RUNTIME parameter, so it does not affect '
          f'the cache)', flush=True)
    if args.dry_run:
        return 0
    for n in sizes:
        build(n, **opts)
    print('\n  Geometry (rho, cable length) is a RUNTIME parameter, so these solvers '
          'are\n  valid for any attachment layout at this fleet size.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
