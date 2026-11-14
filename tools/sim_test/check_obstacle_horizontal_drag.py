#!/usr/bin/env python3
"""OB13: reproduce the frozen OB12 circle discriminator using production code.

Offline only (no ROS, radios or Gazebo). The plant drag/bias are hypothetical,
not uniquely identified physics. No tuning sweep or physical-performance claim.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
from wesley_drone_control.acados import (generate_ocp_controller, set_initial_guess,
    set_planner_reference, warm_start_from_previous_solution)
from wesley_drone_control.command_execution import CommandExecution
from wesley_drone_control.dynamics import QuadLoadDynamics
from wesley_drone_control.horizontal_drag import solver_variant
from wesley_drone_control.xy_integral import XYIntegral, ocp_dimensions
from thesis_c_experiments.trajectories import CaseMotion


def fit_circle(xy):
    b = np.linalg.lstsq(np.c_[2*xy, np.ones(len(xy))], (xy*xy).sum(axis=1), rcond=None)[0]
    return b[:2], float(np.sqrt(b[2]+b[:2]@b[:2]))


def check(repo, output):
    repo, output = Path(repo).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    physical_run = repo/'logs/independent_drones/obstacle_circle_20261029_134832'
    # Missing evidence is an error, never replaced with the current mutable YAML.
    config = json.loads((physical_run/'resolved_config.json').read_text())
    motion = CaseMotion(config['case'], config['path_role'], 0.)
    log = np.genfromtxt(physical_run/'drone2/tracker/log.csv', delimiter=',', names=True)
    heading = float(2*np.arctan2(log['qref_z'][0], log['qref_w'][0]))
    parameters = np.array([42., 0., .12, 100., 100., .5])
    p9 = np.r_[parameters, 0., 0., 0.]
    # Production physical model for the deliberately mismatched plant too.
    plant = QuadLoadDynamics(.38).quad_dynamics()
    dt, hover_s, duration_s = .02, 5., 30.
    references = []
    for step in range(int((duration_s+hover_s)/dt)):
        if step % 5 == 0:
            horizon = np.array([motion.state(step*dt-hover_s+.1*k) for k in range(21)])
            pp, vv = horizon[:, 0, :], horizon[:, 1, :]
            aa = horizon[:, 2, :]+[0., 0., 9.81]
        grid = np.arange(21)*.1
        references.append(tuple(np.stack([np.interp(grid+(step % 5)*dt, grid, a[:, j])
            for j in range(3)], axis=1) for a in (pp, vv, aa)))
    report = dict(scope='Frozen OB12 numerical discrimination, not physical validation',
        physical_reference_run=str(physical_run.relative_to(repo)),
        dt_s=dt, prehover_s=hover_s, duration_motion_s=duration_s, scoring_motion_s=[5., 30.],
        fixed_parameters=parameters.tolist(), integral_weights=[15., 30.], integral_limit=.30,
        plant_body_drag_per_s=.38, plant_world_bias_mps2=[-.54, .24, 0.], trial_drag_per_s=.32,
        limitation='Hypothesized drag/bias plant; fits do not uniquely identify aerodynamics',
        frozen_harness_note='OB12 explicitly zeroed the tiny throttle-reference cost (2e-4 in production); retained here for exact comparison, no runtime cost change',
        source_sha256={str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [repo/'src/wesley_drone_control/wesley_drone_control/acados.py',
                      repo/'src/wesley_drone_control/wesley_drone_control/dynamics.py',
                      physical_run/'resolved_config.json']}, cache_loads=[], variants={})
    previous_cwd = Path.cwd()
    all_data = {}
    try:
        for enabled in (False, True):
            for drag in (0., .32):
                variant = solver_variant(enabled, drag)
                cache = output/'solvers'/variant['cache_subdir']
                cache.mkdir(parents=True, exist_ok=True)
                os.chdir(cache)
                solver = generate_ocp_controller(dynamics=QuadLoadDynamics(drag), xy_integral_enabled=enabled)
                if drag:
                    (cache/'model_options.json').write_text(json.dumps(variant, indent=2)+'\n')
                loaded = generate_ocp_controller(dynamics=QuadLoadDynamics(drag),
                    xy_integral_enabled=enabled, generate=False, build=False)
                manifest = json.loads((cache/(variant['model_name']+'_ocp.json')).read_text())
                assert all(manifest['dims'][k] == v for k, v in ocp_dimensions(enabled).items())
                assert loaded.get(0, 'x').shape == (19 if enabled else 17,)
                report['cache_loads'].append(dict(**variant, dimensions=ocp_dimensions(enabled), repeated_load=True))
                del loaded
                if not enabled:
                    del solver
                    continue
                label = 'drag_model_corrected' if drag else 'baseline'
                W, We = solver.acados_ocp.cost.W.copy(), solver.acados_ocp.cost.W_e.copy()
                W[11, 11] = We[11, 11] = 0.  # Frozen OB12 harness convention, see report.
                for k in range(20): solver.cost_set(k, 'W', W)
                solver.cost_set(20, 'W', We)
                x = np.zeros(17); x[:3] = references[0][0][0]
                x[3] = np.cos(heading/2); x[6] = np.sin(heading/2); x[15] = 9.81/42.
                applied = x[13:17].copy()
                memory, execution = XYIntegral(.30), CommandExecution()
                rows, solve_ms, reference_ms, failures = [], [], [], []
                max_slew = np.zeros(4)
                for step, (p, v, a) in enumerate(references):
                    now = step*dt
                    drag_ff = np.zeros(3)
                    start = time.perf_counter()
                    set_planner_reference(solver, p, v, a, 20, parameters, heading=heading,
                        terminal_vel_ref=True, xy_integral_enabled=True,
                        horizontal_drag_per_s=drag, drag_ff_out=drag_ff)
                    reference_ms.append(1000*(time.perf_counter()-start))
                    state = x.copy(); state[13:17] = applied
                    initial = np.r_[state, memory.begin(now, p[0, :2], x[:2], active=True)]
                    solver.set(0, 'lbx', initial); solver.set(0, 'ubx', initial)
                    if not step: set_initial_guess(solver)
                    else: warm_start_from_previous_solution(solver)
                    start = time.perf_counter(); status = solver.solve()
                    solve_ms.append(1000*(time.perf_counter()-start)); memory.finish(status == 0)
                    if status:
                        failures.append(dict(step=step, status=int(status)))
                        break
                    command, elapsed = execution.evaluate(applied, solver.get(1, 'x')[13:17], now)
                    assert np.isfinite(command).all() and np.max(np.abs(command)) <= 1.000001
                    assert .049999 <= command[2] <= .600001
                    assert abs(command[2]-applied[2]) <= .5*elapsed+1e-5
                    max_slew = np.maximum(max_slew, np.abs(command-applied)/max(elapsed, 1e-9))
                    applied = np.array([round(command[0], 3), round(command[1], 3),
                        (round(2*command[2]-1, 3)+1)/2, round(command[3], 3)])
                    execution.published(now); x[13:17] = applied
                    def f(xx):
                        derivative = np.asarray(plant(xx, np.zeros(4), p9)).ravel()
                        derivative[7:10] += [-.54, .24, 0.]
                        return derivative
                    k1 = f(x); k2 = f(x+.5*dt*k1); k3 = f(x+.5*dt*k2); k4 = f(x+dt*k3)
                    x += dt/6*(k1+2*k2+2*k3+k4); x[3:7] /= np.linalg.norm(x[3:7])
                    assert np.isfinite(x).all()
                    rows.append([now-hover_s, *state[:3], *p[0], *state[7:10],
                        *memory.value, *memory.saturated, *applied, *drag_ff])
                data = np.asarray(rows); all_data[label] = data
                np.savetxt(output/(label+'.csv'), data, delimiter=',', comments='',
                    header='time,x,y,z,rx,ry,rz,vx,vy,vz,xi_x,xi_y,clip_x,clip_y,roll,pitch,throttle,yaw,drag_ff_x,drag_ff_y,drag_ff_z')
                scored = data[data[:, 0] >= 5.]
                centre, radius = fit_circle(scored[:, 1:3]); rc, rr = fit_circle(scored[:, 4:6])
                errors = np.linalg.norm(scored[:, 1:3]-scored[:, 4:6], axis=1)
                report['variants'][label] = dict(xy_median_m=float(np.median(errors)),
                    xy_p95_m=float(np.percentile(errors, 95)), fitted_radius_m=radius,
                    reference_radius_m=rr, radius_deficit_m=abs(radius-rr),
                    centre_error_m=float(np.linalg.norm(centre-rc)),
                    clip_fraction=np.mean(scored[:, 12:14], axis=0).tolist(), failed_solves=failures,
                    command_slew_max=max_slew.tolist(), finite_bounded_commands=True,
                    solve_ms_median=float(np.median(solve_ms)), solve_ms_p95=float(np.percentile(solve_ms, 95)),
                    reference_ms_p95=float(np.percentile(reference_ms, 95)))
                print(label, json.dumps(report['variants'][label]), flush=True)
                del solver
        baseline, trial = (report['variants'][k] for k in ('baseline', 'drag_model_corrected'))
        report['xy_improvement_fraction'] = 1-trial['xy_median_m']/baseline['xy_median_m']
        report['radius_deficit_improvement_fraction'] = 1-trial['radius_deficit_m']/baseline['radius_deficit_m']
        report['passed'] = bool(report['xy_improvement_fraction'] >= .5 and
            report['radius_deficit_improvement_fraction'] >= .5 and
            not baseline['failed_solves'] and not trial['failed_solves'])
    finally:
        os.chdir(previous_cwd)
        (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for label, data in all_data.items():
        data = data[data[:, 0] >= 5.]
        axes[0].plot(data[:, 1], data[:, 2], label=label)
        axes[1].plot(data[:, 0], 100*np.linalg.norm(data[:, 1:3]-data[:, 4:6], axis=1), label=label)
    axes[0].plot(data[:, 4], data[:, 5], 'k--', label='Saved accepted cubic')
    axes[0].axis('equal'); axes[0].set(xlabel='X (m)', ylabel='Y (m)')
    axes[1].set(xlabel='Motion time (s)', ylabel='XY error (cm)')
    for ax in axes: ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.suptitle('Production fixed-drag trial — numerical evidence only')
    fig.tight_layout(); fig.savefig(output/'comparison.png', dpi=160); plt.close(fig)
    print(json.dumps(report, indent=2), flush=True)
    return report['passed']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    raise SystemExit(0 if check(args.repo, args.output) else 1)
