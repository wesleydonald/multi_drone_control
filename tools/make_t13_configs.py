#!/usr/bin/env python3
"""tools/make_t13_configs.py -- the twin configs for the 13 Nov rig trials (Wesley and Te-Jen, 12 Nov): QUAD4 starts on
the floor holding the object at 9, 1 or 5 o'clock (1.9 m out), the carriers on 3/7/11 lift the ring to 1.2 m and
hover, circle (r 0.3 m, 0.125 m/s) or fly the figure-eight (the rig's 0.75 m, 0.125 m/s); his loaded climb, the
settle drop into the (moving) net with the 3 cm release, the ring stopping for his re-attach on 9 (bands 5/5 cm), the
four flying on; LAND 30 s after the weld. Variants: line_up:=true (as flown) and line_up:=gap (the nearest carrier gap
for the drop transit) at 1 and 5 o'clock; two figure-eight take-back checks (a forced give-up while paused; LAND during
the moving drop). Written from configs/experiments/rig_fm9hc_settle_twin.yaml.

    python3 tools/make_t13_configs.py      # writes configs/experiments/t13_*_twin.yaml, prints the names
"""
import copy
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXP = os.path.join(ROOT, 'configs', 'experiments')
BASE = os.path.join(EXP, 'rig_fm9hc_settle_twin.yaml')
WORLDS = {h: (f'three_rigid_ground_rig_attach_held_pickup_s{h}_L100.sdf', None) for h in (9, 1, 5)}   # 1.9 m out


def ball_start(hour):
    """The ball on QUAD4's tip at spawn (tools/make_held_start_worlds.py): QUAD4 1.9 m out, the tip 0.998847 m along +y."""
    import math
    a = math.radians(-30.0 * hour)
    return [round(1.9 * math.cos(a), 6), round(1.9 * math.sin(a) + 0.998847, 6), 0.08]


TRAJ = {'hover': dict(load_traj='hover'),
        'circle': dict(load_traj='orbit', traj_radius=0.3, traj_speed=0.125, traj_hold_s=5.0),
        'fig8': dict(load_traj='fig_8', fig8_size=0.75, traj_speed=0.125, traj_hold_s=5.0, traj_laps=12,
                     contact_sim_fig8=True)}


def make(traj, start, line_up='true', check=None, cpp=None, tune=None, swing_deg=None, ws=True, infl=0.06, body=None, spline=4, rep_=None, env=None, tti=None, ret=None, climb=None, top=None, tr=False, wref=None, pause=None, landat=None, splice=None):
    c = yaml.safe_load(open(BASE))
    name = (f't13_{traj}_s{start}' + ('_gap' if line_up == 'gap' else '') + (f'_{check}' if check else '')
            + (f'_{cpp}' if cpp else '') + (f'_sw{swing_deg}' if tune else '') + ('' if ws is True else f'_ws{int(round(10 * ws[0]))}z{int(round(10 * ws[1]))}' if ws else '_nows')
            + ('' if infl == 0.06 else f'_inf{int(round(100 * infl))}') + (f'_body{int(round(1000 * body))}' if body else '')
            + ('' if spline == 4 else f'_spl{spline}') + ('_oct' if env == 'outer_octagon' else '') + (f'_tti{tti}' if tti else '') + (f'_ret{ret}' if ret else '') + (f'_climb{climb}' if climb else '') + (f'_top{int(round(10 * top))}' if top else '') + ('_tr' if tr else '') + (f'_wr{int(round(100 * wref))}' if wref else '') + (f'_splice{int(round(100 * splice))}' if splice else '') + (f'_pause{pause}' if pause else '') + ('_gate10' if pause and traj != 'hover' else '') + (f'_land{landat.lower()}' if landat else '')
            + (f'_r{rep_}' if rep_ else '') + '_twin')
    c['name'] = name
    c['world'], c['ball_start'] = WORLDS[start][0], ball_start(start)
    a = c['launch']['args']
    for k in ('load_traj', 'traj_radius', 'traj_speed', 'traj_hold_s', 'fig8_size', 'traj_laps', 'contact_sim_fig8'):
        a.pop(k, None)
    a.update(TRAJ[traj])
    a['pos_limit_m'] = 1.8
    if traj == 'hover':
        a.pop('attach_pause', None)
        a.pop('attach_pause_at', None)
    ev = c['events']
    launch = next(e for e in ev if e['do'] == 'LAUNCH')
    arg = ' '.join(launch['arg'].split())
    for old, new in (('match_xy:=0.08', 'match_xy:=0.05'), ('descent_abort_xy:=0.10', 'descent_abort_xy:=0.05'),
                     ('line_up:=true', f'line_up:={line_up}')):
        assert old in arg, old
        arg = arg.replace(old, new)
    arg += ' drop_release:=0.03'
    if cpp:   # Te-Jen 12 Nov: his C++ planner flies the transit (carrier_model tether, the default), no line-up of ours
        assert ' cpp_authority:=false' in arg and ' line_up:=true' in arg
        arg = arg.replace(' cpp_authority:=false', '').replace(' line_up:=true', '')
        if cpp == 'cppre':
            arg += ' pickup_cpp_reattach:=true'
        if tune:   # Te-Jen 12 Nov, the collision geometry measured on the rig (decisions 12 Nov): swing, spline, inflation, workspace
            import math as _m
            arg += (f' max_swing_angle_rad:={_m.radians(swing_deg):.4f} spline_time_factor:={spline} cooperative_tracking_half_m:={infl}'
                    + ((' workspace_xy:=1.8 workspace_z_max:=2.9' if ws is True else f' workspace_xy:={ws[0]} workspace_z_max:={ws[1]}')
                       if ws else '') + (f' cooperative_body_half_m:={body}' if body else '')
                    + (f' swing_envelope:={env}' if env else '') + (f' target_time_iterations:={tti}' if tti else '')
                    + (f' cpp_max_z:={top}' if top else '') + (' temporal_refinement:=true' if tr else '')
                    + (f' cpp_world_refresh:={wref}' if wref else '') + (f' cpp_splice:={splice}' if splice else ''))
    if check == 'giveup':
        arg = arg.replace('contact_give_up_s:=120', 'contact_give_up_s:=3')
    if ret:   # the supervisor's drop-off (12 Nov): start -> ring -> start, the ring never stops, no re-attach
        arg += f' return_after_drop:={ret}' + (f' return_climb:={climb}' if climb else '')
        a['attach_pause'] = bool(pause)
        a.pop('attach_pause_at', None)
        if pause:   # 13 Nov: the ring pauses at his TAKEOFF and resumes once he is down (PAUSE=takeoff on the rig)
            a['attach_pause_at'] = pause
            a['attach_pause_through_landing'] = True
            if traj != 'hover':   # 13 Nov: his TAKEOFF once the path has run 10 s, as on the rig (R4423-R4433 paused a path not yet started)
                arg += ' start_gate_min_traj_t:=10.0'
    launch['arg'] = arg
    tail = [e for e in ev if e['do'] in ('LAUNCH', 'ARM', 'TAKEOFF', 'WAIT_LIFT')]
    if check == 'giveup':      # his give-up while the figure-eight is paused: the take-back, the path held, LAND
        tail += [dict(t=20.0, do='WAIT_LOG', arg='400 PHASE:TAKEN_BACK'), dict(t=25.0, do='LAND')]   # the sheet: LAND at once
    elif landat:               # 13 Nov (Te-Jen's Codex): our LAND as the abort in this phase of the paused drop-off
        tail += [dict(t=20.0, do='WAIT_LOG', arg=f'500 PHASE:{landat}'), dict(t=21.0, do='LAND')]
    elif ret:                  # back over the start: his LAND lands QUAD4 there, then the fleet's LAND
        tail += [dict(t=20.0, do='WAIT_LOG', arg='500 PHASE:AT_START'), dict(t=21.0, do='HIS_LAND'),
                 dict(t=22.0, do='WAIT_LOG', arg='60 PHASE:LANDED_DISARMED'), dict(t=23.0, do='LAND')]
    elif check == 'landdrop':  # LAND while he flies the object to the moving drop point: the take-back on the move
        tail += [dict(t=20.0, do='WAIT_LOG', arg='300 PHASE:SETTLE_ABOVE_DROP_POINT'), dict(t=21.0, do='LAND')]
    else:
        tail += [dict(t=20.0, do='WAIT_WELD', arg=400), dict(t=50.0, do='LAND')]
    c['events'] = tail
    c['timing']['duration_s'] = 700.0
    what = {'hover': 'the ring hovering', 'circle': 'the ring circling (r 0.3 m, 0.125 m/s)',
            'fig8': 'the ring on the figure-eight (0.75 m, 0.125 m/s)'}[traj]
    c['description'] = (
        f"13 Nov trial twin check (Wesley and Te-Jen 12 Nov; tools/make_t13_configs.py; exploratory, judged after the "
        f"run): QUAD4 holding the object from {start} o'clock 1.9 m out, {what}, "
        + ({'cpp': "HIS C++ PLANNER FLIES THE LOADED TRANSIT (no cpp_authority:=false, no line-up, carrier_model tether)",
            'cppre': "HIS C++ PLANNER FLIES THE LOADED TRANSIT AND THE RE-ATTACH TRANSIT (pickup_cpp_reattach)"}.get(cpp)
           or f"line_up:={line_up}")
        + (", his give-up forced at 3 s (before any contact), LAND 5 s after the take-back (the sheet's rule)" if check == 'giveup' else '')
        + (", LAND as he settles over the moving drop point" if check == 'landdrop' else '')
        + ". Read: QUAD4's and the ball's closest distance to each carrier, the drop (ball in the net), the ring "
        "stopping for the re-attach, the weld, the four flying on; for the checks the path held at rest after the "
        "take-back and no tilt or abort. Falsifier: a carrier passed closer than 0.3 m, the ball not in the net, no "
        "weld in 400 s, the path moving after a figure-eight take-back, an abort.")
    with open(os.path.join(EXP, name + '.yaml'), 'w') as f:
        yaml.safe_dump(c, f, sort_keys=False, width=120)
    return name


if __name__ == '__main__':
    names = []
    for traj in ('hover', 'circle', 'fig8'):
        for start in (9, 1, 5):
            names.append(make(traj, start))
        for start in (1, 5):
            names.append(make(traj, start, 'gap'))
    names += [make('fig8', 9, check='giveup'), make('fig8', 9, check='landdrop')]
    for start in (9, 1, 5):                            # the tuned set (swing 5 deg, spline 4, inflation 0.06, workspace)
        names.append(make('hover', start, cpp='cpp', tune=True, swing_deg=5))
    names.append(make('circle', 9, cpp='cpp', tune=True, swing_deg=5))
    for sw in (3, 7):                                  # Te-Jen's sweep on the two quickest cases
        names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=sw))
        names.append(make('circle', 9, cpp='cpp', tune=True, swing_deg=sw))
    for start in (9, 1, 5):                            # the same without the workspace (R4366-R4373: no plan from a 1.9 m start)
        names.append(make('hover', start, cpp='cpp', tune=True, swing_deg=5, ws=False))
    names.append(make('circle', 9, cpp='cpp', tune=True, swing_deg=5, ws=False))
    names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=3, ws=False))
    # Te-Jen's overnight sweep, one variable at a time from the no-workspace swing-5 set, on the hover from 9 o'clock
    names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, infl=0.04))
    names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, body=0.09))
    names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, spline=6))
    names.append(make('circle', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, rep_=2))     # R4377 repeated
    # 12 Nov night: his exact convex components instead of his boxes (the boxes put the hover drop point inside carriers 7/11)
    for start in (9, 1, 5):
        names.append(make('hover', start, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon'))
    for traj in ('circle', 'fig8'):
        names.append(make(traj, 9, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon'))
    # and his rendezvous-time fixed point given 30 steps (the moving drop: TARGET_TIME_FIXED_POINT_NOT_CONVERGED at spline 4)
    for start in (9, 1, 5):
        names.append(make('circle', start, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30))
    names.append(make('fig8', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30))
    names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30))
    # the supervisor's drop-off (12 Nov): fly in, drop, fly back to the start, the ring moving throughout
    for traj, start in (('fig8', 9), ('circle', 5), ('hover', 9)):
        names.append(make(traj, start, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='cpp'))
    names.append(make('fig8', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct'))
    # the sheet's other drop-offs with the direct return (13 Nov)
    names.append(make('hover', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct'))
    names.append(make('circle', 5, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct'))
    # his search box capped at 2.9 m body z (13 Nov: he climbed to 3.2-3.6 m on a moving ring; the rig keeps QUAD4 under 2.8-3.0)
    for traj, start in (('fig8', 9), ('circle', 5), ('hover', 9)):
        names.append(make(traj, start, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct',
                          top=2.9))
    # 13 Nov (Wesley: his planner lower, the ring speed unchanged): capped at 2.8 m, then each lever on its own and together
    kw = dict(cpp='cpp', tune=True, ws=False, env='outer_octagon', tti=30, ret='direct', top=2.8)
    names.append(make('fig8', 9, swing_deg=5, **kw))
    names.append(make('fig8', 9, swing_deg=5, spline=3, **kw))
    names.append(make('fig8', 9, swing_deg=5, infl=0.04, **kw))
    names.append(make('fig8', 9, swing_deg=3, **kw))
    names.append(make('fig8', 9, swing_deg=3, spline=3, infl=0.04, **kw))
    names.append(make('circle', 5, swing_deg=3, spline=3, infl=0.04, **kw))
    # spline 2 (planning intervals ~0.5-0.75 s near the ring: the probe on 12 Nov's flights left the drop point at 2.76 m free
    # 90-100 % of the time at 0.5 s), with swing 3 and inflation 0.04, capped at 2.8; and the 3.8 m cap alone (Wesley's fallback)
    names.append(make('fig8', 9, swing_deg=3, spline=2, infl=0.04, **kw))
    names.append(make('circle', 5, swing_deg=3, spline=2, infl=0.04, **kw))
    names.append(make('hover', 9, swing_deg=3, spline=2, infl=0.04, **kw))
    names.append(make('fig8', 9, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct', top=3.8))
    names.append(make('circle', 5, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct', top=3.8))
    # his temporal refinement on (Te-Jen's Codex 13 Nov 01:41: isolated, everything else as R4400 / R4390 / batch 5's top28)
    base = dict(cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='direct')
    for traj, start in (('fig8', 9), ('circle', 5)):
        names.append(make(traj, start, tr=True, **base))
        names.append(make(traj, start, top=2.8, tr=True, **base))
    # tomorrow's CPP=on line exactly (cap 3.8, world refresh 0.2), hover from 9: the direct return and the C++ return (climb)
    full = dict(cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, top=3.8, wref=0.2)
    names.append(make('hover', 9, ret='direct', **full))
    names.append(make('hover', 9, ret='cpp', climb='transit', **full))
    names.append(make('hover', 9, ret='direct', rep_=2, **full))
    names.append(make('fig8', 9, ret='direct', **full))
    names.append(make('circle', 5, ret='direct', **full))
    # PAUSE=takeoff: his C++ drop-off over a still ring inside the figure-eight / circle experiment (13 Nov, after the
    # real-time replays: over a moving ring he never reached the drop point)
    names.append(make('fig8', 9, ret='direct', pause='takeoff', **full))
    names.append(make('fig8', 9, ret='direct', pause='takeoff', rep_=2, **full))
    names.append(make('fig8', 9, ret='cpp', climb='transit', pause='takeoff', **full))
    names.append(make('circle', 5, ret='direct', pause='takeoff', **full))
    # 13 Nov ~02:30: the swing bound at 3 deg lets him in over the MOVING ring without climbing (twin R4420/R4421);
    # tomorrow's line with it (cap 2.9, refresh 0.2), figure-eight and circle, twice each, and with spline 3
    s3 = dict(cpp='cpp', tune=True, swing_deg=3, ws=False, env='outer_octagon', tti=30, top=2.9, wref=0.2, ret='direct')
    names.append(make('fig8', 9, **s3))
    names.append(make('fig8', 9, rep_=2, **s3))
    names.append(make('circle', 5, **s3))
    names.append(make('circle', 5, rep_=2, **s3))
    names.append(make('fig8', 9, spline=3, **s3))
    # our LAND as the abort over the paused figure-eight: in his C++ transit, at the drop, in the direct return
    for ph in ('TRANSIT_TO_DROP_POINT', 'CLEAR_DROP_ZONE', 'RETURN_TO_START'):
        names.append(make('fig8', 9, ret='direct', pause='takeoff', landat=ph, **full))
    # the same with his search box capped at 2.9 m (real-time replays over a still ring: captured at 2.85 m)
    full29 = dict(full, top=2.9)
    names.append(make('fig8', 9, ret='direct', pause='takeoff', **full29))
    names.append(make('circle', 5, ret='direct', pause='takeoff', **full29))
    names.append(make('hover', 9, ret='direct', **full29))
    names.append(make('fig8', 9, ret='direct', pause='takeoff', rep_=2, **full29))
    names.append(make('circle', 5, ret='direct', pause='takeoff', rep_=2, **full29))
    # the worst-case backup (Wesley 13 Nov): no C++ at all, his Python transfer both ways, the line-up as on 12 Nov
    names.append(make('fig8', 9, ret='direct'))
    # the C++ return's hand-over at the transit height (13 Nov: in the usual 3 cm climb it sank onto his search floor, R4406)
    for traj, start in (('hover', 9), ('fig8', 9)):
        names.append(make(traj, start, cpp='cpp', tune=True, swing_deg=5, ws=False, env='outer_octagon', tti=30, ret='cpp',
                          climb='transit'))
    # his height capped by his workspace box (a 1.9 m start inside +-2.4; ceiling 3.0 m on the whole assembly: body <= ~2.89 m; 2.9 left the 2.8 m hold outside, R4401)
    names.append(make('fig8', 9, cpp='cpp', tune=True, swing_deg=5, ws=(2.4, 3.0), env='outer_octagon', tti=30, ret='cpp'))
    for traj in ('hover', 'circle', 'fig8'):          # Te-Jen 12 Nov: his C++ planner for the transit
        for start in (9, 1, 5):
            names.append(make(traj, start, cpp='cpp'))
        names.append(make(traj, 1, cpp='cppre'))
    print('\n'.join(names))
