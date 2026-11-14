#!/usr/bin/env python3
"""OB7 real-solver bias discriminator. Offline only: no ROS/radios/Gazebo."""
import argparse
import json
import os
from pathlib import Path
import time

import casadi as ca
import numpy as np

from wesley_drone_control.acados import (generate_ocp_controller,
    set_initial_guess, set_planner_reference, warm_start_from_previous_solution)
from wesley_drone_control.command_execution import CommandExecution
from wesley_drone_control.dynamics import QuadLoadDynamics
from wesley_drone_control.xy_integral import XYIntegral, ocp_dimensions


def check(directory):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    model = QuadLoadDynamics()
    physical = model.quad_dynamics()
    params = np.array([42., 0., .12, 100., 100., .5])
    p9 = np.r_[params, 0., 0., 0.]
    positions = np.tile([0., 0., 1.], (21, 1))
    velocity = np.zeros((21, 3))
    acceleration = np.tile([0., 0., 9.81], (21, 1))
    report = dict(plant_lateral_disturbance_mps2=[0., .4], duration_s=35.,
                  dt_s=.02, integral_weights=[15., 30.], integral_limit_ms=.30,
                  scope='Known-bias numerical plant; not a physical validation', variants={})
    start_cwd = Path.cwd()
    try:
        for enabled in (False, True):
            label = 'integral' if enabled else 'legacy'
            cache = directory / label
            cache.mkdir()
            os.chdir(cache)
            solver = generate_ocp_controller(xy_integral_enabled=enabled)
            # Exercise repeat cache loading, not just construction.
            loaded = generate_ocp_controller(xy_integral_enabled=enabled, generate=False, build=False)
            assert loaded.get(0, 'x').shape == (19 if enabled else 17,)
            del loaded
            manifest = json.loads((cache / (solver.acados_ocp.model.name+'_ocp.json')).read_text())
            assert all(manifest['dims'][k] == v for k, v in ocp_dimensions(enabled).items())
            memory, execution = XYIntegral(), CommandExecution()
            x = np.zeros(17); x[2] = 1.; x[3] = 1.; x[15] = 9.81/42.
            applied = x[13:17].copy()
            rows, costs, failures = [], [], 0
            for step in range(1750):
                now = step*.02
                set_planner_reference(solver, positions, velocity, acceleration, 20, params,
                    terminal_vel_ref=True, xy_integral_enabled=enabled)
                state = x.copy(); state[13:17] = applied
                if enabled:
                    state = np.r_[state, memory.begin(now, positions[0, :2], x[:2], active=True)]
                solver.set(0, 'lbx', state); solver.set(0, 'ubx', state)
                if not step:
                    set_initial_guess(solver)
                else:
                    warm_start_from_previous_solution(solver)
                started = time.perf_counter()
                status = solver.solve()
                costs.append((time.perf_counter()-started)*1000.)
                if enabled:
                    memory.finish(status == 0)
                failures += int(status != 0)
                if status:
                    raise RuntimeError(f'{label}: solve status {status} at step {step}')
                command, elapsed = execution.evaluate(applied, solver.get(1, 'x')[13:17], now)
                assert np.isfinite(command).all() and 0. <= command[2] <= .600001
                assert abs(command[2]-applied[2]) <= .5*elapsed+1e-6
                applied = np.array([round(command[0], 3), round(command[1], 3),
                                    (round(2*command[2]-1, 3)+1)/2, round(command[3], 3)])
                execution.published(now)
                x[13:17] = applied

                def f(xx):
                    result = np.asarray(physical(xx, np.zeros(4), p9)).ravel()
                    result[8] += .4  # Unmodelled plant-only acceleration; never supplied to MPC.
                    return result

                k1 = f(x); k2 = f(x+.01*k1); k3 = f(x+.01*k2); k4 = f(x+.02*k3)
                x += .02/6*(k1+2*k2+2*k3+k4)
                x[3:7] /= np.linalg.norm(x[3:7])
                rows.append([now, *x[:3], *memory.value, *command])
            data = np.asarray(rows)
            np.savetxt(cache/'closed_loop.csv', data, delimiter=',',
                       header='time_s,x,y,z,xi_x,xi_y,roll,pitch,throttle,yaw', comments='')
            tail = data[data[:, 0] >= 30.]
            report['variants'][label] = dict(
                final5s_median_xy_m=float(np.median(np.linalg.norm(tail[:, 1:3], axis=1))),
                final5s_z_bias_m=float(np.median(tail[:, 3]-1.)),
                failed_solves=failures, solver_ms_median=float(np.median(costs)),
                solver_ms_p95=float(np.percentile(costs, 95)),
                final_integral=memory.value.tolist(), dimensions=ocp_dimensions(enabled),
                repeated_load=True)
        baseline = report['variants']['legacy']['final5s_median_xy_m']
        updated = report['variants']['integral']['final5s_median_xy_m']
        report['passed'] = bool(updated < .05 and updated <= .5*baseline)
    finally:
        os.chdir(start_cwd)
        (directory/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    result = check(parser.parse_args().output)
    raise SystemExit(0 if result['passed'] else 1)
