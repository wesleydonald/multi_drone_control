"""The C++ exit's twin wiring checks (handoff 15 Nov; plan v2 section 7): six configs from the 13 Nov drop-off twin lines
with the exit trial line's switches (return_after_drop:=cpp exit_handoff:=moving max_body_z:=3.2 cpp_max_z:=3.2; R4527-R4529 flew 3.3).
Writes configs/experiments/t15x_*.yaml. Wiring only: the twin runs at 0.13-0.16x real time."""
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
EXP = REPO / 'configs' / 'experiments'
FIG8 = (EXP / 't15_fig8_s9_cpp_sw3_spl2_retdirect_endless_twin.yaml').read_text()
CIRCLE = (EXP / 't13_circle_s5_cpp_sw3_nows_inf4_spl2_oct_tti30_retdirect_top28_twin.yaml').read_text()
BASE_EVENTS = '''- t: 20.0
  do: WAIT_LOG
  arg: 500 PHASE:AT_START
- t: 21.0
  do: HIS_LAND
- t: 22.0
  do: WAIT_LOG
  arg: 60 PHASE:LANDED_DISARMED
- t: 23.0
  do: LAND
'''
END = '''- t: 40.0
  do: WAIT_LOG
  arg: 200 PHASE:LANDED_DISARMED
- t: 41.0
  do: LAND
'''
EVENTS = {
    # the normal exit: AT_START -> LANDING after 2 s on its own; the fleet LAND once QUAD4 is LANDED_DISARMED
    'normal': '''- t: 20.0
  do: WAIT_LOG
  arg: 500 PHASE:AT_START
''' + END,
    # his LAND while the exit prepares (the clearance done, no grant yet): the escape
    'landprep': '''- t: 20.0
  do: WAIT_LOG
  arg: 500 HIS:C.. exit. clearance complete
- t: 20.01
  do: HIS_LAND
''' + END,
    # his LAND after the grant, under C++ authority: the escape
    'landgrant': '''- t: 20.0
  do: WAIT_LOG
  arg: 500 PHASE:RETURN_TO_START
- t: 20.3
  do: HIS_LAND
''' + END,
    # the 7 s preparation timeout with his backend silent (frozen at the clearance's completion, resumed after the
    # failure): the failure at +7 s, back on the live ring-following; his LAND 10 s later: the escape
    'timeout': '''- t: 20.0
  do: WAIT_LOG
  arg: 500 HIS:C.. exit. clearance complete
- t: 20.0
  do: SIGNAL
  arg: STOP dynamic_planner_transfer_backend
- t: 21.0
  do: WAIT_LOG
  arg: 120 HIS:C.. exit hand-off FAILED
- t: 23.0
  do: SIGNAL
  arg: CONT dynamic_planner_transfer_backend
- t: 31.0
  do: HIS_LAND
''' + END,
    # the fleet LAND during QUAD4's descent at its start
    'landdesc': '''- t: 20.0
  do: WAIT_LOG
  arg: 500 PHASE:LANDING
- t: 21.0
  do: LAND
- t: 22.0
  do: WAIT_LOG
  arg: 200 PHASE:LANDED_DISARMED
''',
}
RUNS = [  # name, base, trajectory, events, extra launch args, what it checks
    ('t15x_hover_normal', 'fig8', 'hover', 'normal', '', 'hover first: the exit end to end, AT_START -> LANDING, then the fleet LAND after LANDED_DISARMED'),
    ('t15x_hover_landprep', 'fig8', 'hover', 'landprep', '', 'his LAND while the exit prepares: Cancel + enable=false, the escape (direct return), LANDING'),
    ('t15x_hover_landgrant', 'fig8', 'hover', 'landgrant', '', 'his LAND after the grant (C++ flying): the escape, LANDING'),
    ('t15x_fig8_timeout', 'fig8', 'fig8', 'timeout', '', 'the 7 s preparation timeout (his backend frozen from the clearance\'s completion, resumed after the failure): the failure, back on the live ring-following (no stop, no Python return); his LAND 10 s later: the escape'),
    ('t15x_fig8_landdesc', 'fig8', 'fig8', 'landdesc', '', 'the endless figure-eight exit end to end; the fleet LAND during QUAD4\'s descent'),
    ('t15x_circle_normal', 'circle', 'circle', 'normal', '', 'the circle exit end to end, then the fleet LAND after LANDED_DISARMED'),
]


def make(name, base, traj, events, extra, what, world14=False):
    text = FIG8 if base == 'fig8' else CIRCLE
    first = text.split('\n', 1)[0]
    text = text.replace(first, f'name: {name}', 1)
    text = re.sub(r"description: '.*?'\n(?=world:)", lambda m: (
        f"description: '15 Nov C++ exit twin wiring check (handoff 15 Nov, plan v2 section 7; exploratory, wiring only, "
        f"not timing): {what.replace(chr(39), chr(39) * 2)}. The exit trial line: return_after_drop:=cpp exit_handoff:=moving max_body_z:=3.2 "
        f"cpp_max_z:=3.2 on {('t15_fig8_s9_cpp_sw3_spl2_retdirect_endless_twin' if base == 'fig8' else 't13_circle_s5_cpp_sw3_nows_inf4_spl2_oct_tti30_retdirect_top28_twin')}"
        f"{' with the ring hovering' if traj == 'hover' else ''}. Falsifier: no AT_START or LANDED_DISARMED, a stationary seed, "
        f"a reference step over 5 cm at a switch, QUAD4 over 3.2 m, an abort.'\n"), text, count=1, flags=re.S)
    if traj == 'hover':
        text = re.sub(r'    load_traj: fig_8\n    fig8_size: 0.75\n    traj_speed: 0.125\n    traj_hold_s: 5.0\n'
                      r'    traj_laps: 0\n    contact_sim_fig8: true\n', '    load_traj: hover\n', text)
        assert 'load_traj: hover' in text
    if base == 'fig8':
        text = text.replace('max_body_z:=2.8', 'max_body_z:=3.2', 1).replace(
            'cpp_max_z:=2.8 return_after_drop:=direct', 'cpp_max_z:=3.2 return_after_drop:=cpp exit_handoff:=moving' + extra, 1)
    else:
        text = text.replace('max_body_z:=2.8', 'max_body_z:=3.2', 1)
        text = re.sub(r'cpp_max_z:=2\.8 return_after_drop:=direct', 'cpp_max_z:=3.2 return_after_drop:=cpp exit_handoff:=moving'
                      + extra, text, count=1)
    if world14:
        # QUAD4 from (0, 1.4) on plate 9's line (the held-object world): its start hover, the exit's goal, inside the
        # exit's assembly centre bounds (+-1.61 m); 1.9 m out (s9, s5) it lies outside and his planner holds at the edge
        text = re.sub(r'world: three_rigid_ground_rig_attach_held_pickup_s[0-9]_L100\.sdf',
                      'world: three_rigid_ground_rig_attach_held_pickup_L100.sdf', text, count=1)
        text = re.sub(r'ball_start:\n(- .*\n){3}', 'ball_start:\n- 0.0\n- 2.399\n- 0.08\n', text, count=1)
        assert 'held_pickup_L100.sdf' in text and '- 2.399' in text, name
    assert 'exit_handoff:=moving' in text and 'max_body_z:=3.2' in text, name
    assert BASE_EVENTS in text, name
    text = text.replace(BASE_EVENTS, EVENTS[events], 1)
    (EXP / f'{name}.yaml').write_text(text)
    return EXP / f'{name}.yaml'


# ~19:20: R4527 (s9 world) held at the bound's edge, its start 1.9 m out: the same checks from (0, 1.4)
RUNS14 = [(n + '_s14', b, tr, ev, ex, w + '; QUAD4 from (0, 1.4) (the held-object world), its start inside the exit bounds')
          for n, b, tr, ev, ex, w in RUNS]

# ~21:05 (Te-Jen and Codex's closeout): R4535's circle refused 42 safe candidates WORLD_CHANGED_DURING_FINAL_RECHECK, his
# world rebuilt on every message (9 versions per ~290 ms replan): these lines lacked the trial line's backend values,
# which the rig script types for every CPP=on line (WR 0.2 since 13 Nov, splice 0.30, tracking 0.06, the settle
# bounds). The two remaining checks fly them; the timeout check also raises his twin supervisor's backend-status limit
# for that run only (the frozen backend went quiet ~50 s wall; R4533's disarm)
TRIAL = (' cpp_world_refresh:=0.2 cpp_splice:=0.30 prepare_settle_speed:=0.20 prepare_settle_accel:=0.8')
EVENTS['normal_or_land'] = '''- t: 20.0
  do: WAIT_LOG
  arg: 120 PHASE:AT_START
- t: 25.0
  do: HIS_LAND
''' + END
RUNS_TRIAL = [
    ('t15x_circle_trial_s14', 'circle', 'circle', 'normal_or_land', TRIAL,
     'the circle exit on the trial line\'s backend values (R4535 without them); his LAND 5 s after AT_START, or 125 s '
     'after it never comes (the escape)'),
    ('t15x_fig8_timeout_trial_s14', 'fig8', 'fig8', 'timeout', TRIAL + ' supervisor_status_timeout_s:=120',
     'the 7 s preparation timeout with his backend frozen from the clearance\'s completion and his twin supervisor\'s '
     'backend-status limit 120 s for this run (R4533 disarmed at 15 s): the failure, the cancel blend to the live '
     'ring-following reference, his LAND 10 s later (the escape)'),
]

if __name__ == '__main__':
    # RUNS flew as written at 18:50 (R4527, R4528: cpp_max_z 3.3); from the review of 15 Nov the exit line's loaded
    # search top is its 3.2 m body limit (the launch refuses 3.3 there), so only RUNS14 are written now, at 3.2
    for r in RUNS14:
        print(make(*r, world14=True))
    for r in RUNS_TRIAL:
        path = make(*r, world14=True)
        text = path.read_text()
        assert text.count('cooperative_tracking_half_m:=0.04') == 1
        path.write_text(text.replace('cooperative_tracking_half_m:=0.04', 'cooperative_tracking_half_m:=0.06', 1))
        print(path)
