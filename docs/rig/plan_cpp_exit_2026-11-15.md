# Plan: Te-Jen's C++ planner flies the exit after the drop-off (draft for the co-plan, 15 Nov 2026)

For Te-Jen and his Codex, from Wesley and the lab chat. **No code changes until we agree on this plan.**
Numbers marked *(ours)* are the lab chat's exploratory readings of the 13 Nov rig logs. They have not been reviewed.

## 1. The aim and the rules
- **The aim.** After the drop into the net, his C++ planner flies QUAD4 out of the ring and back to its start. Today our
  Python transfer flies the exit (`RET=direct`), and his planner flies the approach and the drop.
- **Rules:**
  - The real world comes first; simulation checks the wiring only.
  - One rig flight decides: if it does not work, we do not use it (Te-Jen).
  - There is a backup if his planner cannot plan.
  - Nothing bypasses his planner without his agreement (Te-Jen, 13 Nov).
  - Tuesday's nine drop-offs for the paper fly the frozen line (`RET=direct`) first. The C++ exit is a separate cohort,
    flown after them.

## 2. What exists today
`return_after_drop:=cpp` (`RET=cpp` in the rig scripts) is already wired, but it has never flown on the rig.

**The sequence after the release** (all in `src/tejen_mission/tejen_mission/online_join_planner.py`):
1. DROP_OBJECT, then CLEAR_DROP_ZONE: a climb relative to the ring (8866-8879). On the low route it is only the release
   height + 3 cm.
2. The branch between direct and cpp is at 8881-8897.
3. With cpp, `c1f2_climb_hold_handoff(..., exit_phase=RETURN_TO_START)` (8171-8241) freezes QUAD4 at its measured
   position and waits for his stop gate: speed <= 0.20 m/s, accel <= 0.8 m/s^2 on the rig line, dwell 0.3 s.
4. His backend then plans to the start hover, given as a static target (`transfer_backend_node.cpp` 570, 971-974,
   2109). It refuses a start that is not stationary: speed <= 0.10 m/s (2910-2915).
5. His MPC flies QUAD4 the whole time. Only the reference authority changes hands, from Python to C++.

**The twin, 13 Nov:**

| Runs | Ring | Result |
|---|---|---|
| R4406, R4407 | hover / circle | The hold sank ~0.3 m onto his search floor and the return never started |
| R4413 | moving figure-eight | The return never started: RECOVERY_WAITING_COMMON_STOPPED_REFERENCE. His Codex: "no moving-seed handover exists" |
| R4414, R4434, R4427 | still (or paused) | Worked, with `return_climb:=transit`. The return started ~9 s after the drop and reached AT_START ~10 s later |

- So the rig flew `RET=direct` (Wesley 13 Nov 00:55 and 06:31).

**Rig line arithmetic, not yet run:** `CPP=on` with `RET=cpp` would be refused by the launch. With `return_climb`
transit, the planned "clear of the drop" height is about 3.19 m, over the 3.0 m `max_body_z`. Without `return_climb` it
is 2.64 m. A desk launch will confirm this.

## 3. What the 13 Nov rig logs show about the exit
Eight drops flew our Python return. From the release to AT_START took **6.7-9.3 s** each time; that is the baseline to
beat or match. QUAD4's body was 2.37-2.43 m at the release and 2.49-2.55 m when the return started.

| Flight | Ring | Return (s) | Path (m) | Closest QUAD4 cable to a carrier cable in the exit (m) |
|---|---|---|---|---|
| fm9h8_5 | figure-eight | 8.1 | 1.67 | 0.16 |
| fm9hc | circle | 5.9 | 1.15 | 0.20 |
| fm9hc_8 | circle | 7.4 | 1.53 | 0.19 |
| fm9h8_13 | figure-eight | 6.5 | 1.20 | **0.13** |
| fm9h8_14 | figure-eight | 8.2 | 1.85 | 0.22 |
| fm9hc_10 | circle | 7.3 | 1.55 | 0.22 |
| fm9h_2 | hover | 6.7 | 1.43 | 0.23 |
| fm9hc_11 | circle | 8.5 | 1.45 | 0.24 |

**Our Python return flew over a carrier.**
- In fm9h8_13, 5.9 s after the release, QUAD4 (2.63 m) passed 0.13 m horizontally from QUAD1's body (2.13 m): 0.5 m
  directly above it.
- Cause: the return is a straight line to the start, with no carrier check, and the figure-eight moved the ring under
  that line.
- So today's backup breaks the rule that QUAD4 never flies over the carriers. A C++ exit is the proper fix. The Python
  return also needs a fix if it stays as the backup (B0 below).

**The main hazard for a C++ exit over a moving ring** *(ours, from replaying the recorded carriers past a still
QUAD4)*:
- His backend needs QUAD4 stopped before it plans. While QUAD4 holds still at its hand-over pose (~2.5 m), the moving
  carriers' cables come within:
  - 0.04-0.12 m in 3 s, in 3 of 7 moving flights;
  - 0.00-0.07 m in 10 s, in 5 of 7 moving flights.
  - The hover flight stayed at 0.28 m.
- On the rig the approach's stop-gate hand-overs took 0.8-4.8 s. So a stop-and-wait start next to a moving ring is not
  safe.
- At the release, QUAD4's cable was already 0.18-0.24 m from the carriers' cables. This start may lie inside his
  inflated collision set.

## 4. The question to settle first: how the exit starts over a moving ring
| Option | What it is | For | Against |
|---|---|---|---|
| **A. Moving-seed hand-over** (his backend) | His backend accepts a start that moves with the ring, seeded from the commanded reference, like the approach's moving target | His planner flies the whole exit, including the part next to the carriers | A change to his backend's contract; the most work; needs his time |
| **B. Pause the ring at the release** | The carriers hold still from the release until QUAD4 is clear, then the path resumes | His backend as is (R4414/R4434/R4427 worked over a still ring) | Wesley rejected pausing for the drop itself; it changes the "ring never stops" claim; the paused-ring prediction bug (13 Nov 05:16, the bridge predicts the ring stopped ~0.19 m early) applies |
| **C. Short Python leg first, then C++** | Python climbs and moves QUAD4 radially out along plate 9's line, relative to the moving ring, past the carriers (~1.1 m from the centre); his planner flies from there to the start | No change to his backend; quick | His planner does not fly the part next to the carriers, which is where the risk is; it is closer to the backup than to the aim |
| **D. Hover only** | Fly the C++ exit over the hovering ring only | It worked in the twin; the least risk | Does not cover the circle or figure-eight |

The lab chat's view:
- For this week, **D first, then A if Te-Jen can build it**.
- **C** only as the backup's first leg (B0 below).
- B needs Wesley's word.

## 5. The backup (it falls to our Python return in every case)
- **B0: make the Python return safe.**
  - It climbs, then moves out radially along plate 9's bearing relative to the moving ring until clear of the carrier
    circle, then goes straight to the start.
  - This fixes the fm9h8_13 overflight whether or not the C++ exit is used.
  - Our code (the mission's direct reference, 9357-9365, `VirtualTransferGenerator`); Wesley's word.
- **B1: before his planner takes authority.**
  - Trigger: no hand-over ready within T s of the return's preparation, or his status stays WAITING_* or INVALID_*.
    Today there is no timeout and the hold waits forever (21-45 min in the twin).
  - Action: reset the hand-over and fly B0.
  - Where: the cpp branch at 8883-8893.
  - T is to agree; ~2 s over a moving ring, given the sweep times above.
- **B2: after he takes authority.**
  - Extend `_c1f2_python_fallback_step` (5458-5491, today TRANSIT_TO_DROP_POINT only) to RETURN_TO_START.
  - Triggers: his EXECUTION_FALLBACK_* statuses; a hard hover; no progress (QUAD4 under ~0.05 m/s for >= 1-2 s within
    0.5 m of a carrier cable).
  - Measured by the mission at 100 Hz, because his status reaches the mission only at 1 Hz.
- **A backup firing on the rig counts as a C++ failure** (the one-run rule).
- Our LAND and his LAND keep their meaning during the C++ exit. Our LAND during a C++ return was never tested; P1
  tests it.

## 6. Testing, in order
**P0 Offline replay on the laptop, on the 13 Nov logs.**
- The harness exists: `results/analysis/cpp_realtime_2026-11-13/full/` (`run_flight.sh`, `driver.py`). It plays the
  recorded ring and carriers into `ring_bridge` and his real backend binary (the frozen line's, 13 Nov 10:15), with a
  virtual QUAD4 that tracks perfectly.
- Changes for the exit:
  - Point the harness at the 13 Nov bags.
  - Give it the still start-hover goal, with `static_target` true and the object detached.
  - Type the frozen line's backend values: splice 0.30, cap 3.3, search 0.10, state timeout 1.0, brakes off.
- Cases:
  - The eight 13 Nov drops (release instants and seed poses in the lab chat's table).
  - Synthetic releases placed along each flight's moving window, >= 10 per ring motion.
  - 12 Nov's longer circling (nro).
  - After the release the ring flew on only 10-23 s before the LAND, shorter than a ~19 s C++ exit. So the longer
    windows need the synthetic releases.
- Each start option (section 4) is run at idle and with 8 busy cores.
- Pass for a start option on a ring motion:
  - his planner finds a plan every time;
  - QUAD4 never sits still for more than 1 s within 0.5 m of a moving carrier cable;
  - clearances stay above his margins;
  - it never goes over a carrier;
  - it stays inside +-1.8 m and under the height cap.
- One miss rules that option out for that ring motion.
- Limits: open loop (no swing; perfect tracking), as his Codex noted for the approach replays.

**P1 Twin on the lab PC.** Wiring only: the twin runs at 0.13-0.16x real time and hid the real-time faults on 12-13 Nov.
- Configuration: the frozen rig line exactly, with the chosen start, hover first, 2 repeats.
- Forced failures:
  - B1 by timeout;
  - B2 by a hard hover;
  - our LAND during the C++ exit;
  - his LAND at AT_START.
- A critic reads the card first: a new hand-over and fallback in the flight loop.

**P2 Rig: one flight, hover first, after Tuesday's nine.**
- One failure (a backup firing included) = not used.
- The circle and figure-eight follow only if P0 passed for them.
- The sheet's stop rules stay: our LAND if QUAD4 stalls beside the moving ring for ~2 s or its cable heads into a
  carrier's.

**What we measure** (P0-P2, against the Python baseline above):
- Exit time: release, grant, out of the carrier circle, AT_START.
- Closest QUAD4 cable and magnet to each carrier cable and body.
- Any overflight.
- Highest body z.
- Largest |x| and |y|.
- Plan latency: preparation to ready, grant to acknowledgement, replan median/p95 against the 0.30 s splice.
- Failed or late replans; time spent still beside a moving ring; backup triggers.

## 7. Who does what (to agree)
| Te-Jen / Codex | Wesley / lab chat |
|---|---|
| Answer section 8 | The harness port and the P0 runs; report the results |
| If option A: the moving-seed hand-over in his backend | B0, B1 and B2 in the mission (our side of his fork), each logged in CHANGES_FROM_TEJEN.md |
| Check that his collision check accepts the exit start (QUAD4's cable 0.18-0.24 m from the tethers) | The launch: the height cap with `return_climb`, the workspace inside +-1.8 m |
| Review our B1/B2 triggers before they fly | P1, the critic, the rig sheet row |

## 8. Questions for Te-Jen
1. Over a moving ring, which start: A (moving-seed hand-over in your backend), B (pause), C (a Python leg first) or D
   (hover only for now)? Can you build A this week?
2. Does your checker accept a start with QUAD4's cable 0.18-0.24 m from the carriers' tethers, or must QUAD4 first climb
   or move out?
3. The swing bound: 3 deg is set, but the bare magnet swung 4.4-8.5 deg (max) after the release *(ours, 8 flights)*.
   Can the exit use its own bound?
4. The backup: are B1 and B2 acceptable to you, with which triggers and which T? May our Python take over when your
   planner cannot plan?
5. The hand-over height: ~2.5 m (the 3 cm climb) or ~3.0 m (`return_climb` transit, which the launch refuses today
   under the 3.0 m cap)?
6. The search box: top 3.3 m; no `workspace_xy` is typed, so your box can reach y ~2.2 m, past the 1.8 m position
   limit. Should the exit type a workspace?
7. The goal: is the start hover right? During AT_START the circling carriers' cables come within 0.40 m.
8. With `RET=cpp` the launch sets `require_payload_attached_for_enable` false for the whole flight, including your
   loaded approach. Is that acceptable, or should it switch only at the drop?
9. What offline evidence (open-loop replay, no swing) would you accept before the one rig flight?

## Files (in the zip)
- `online_join_planner.py`, `c1f2_handoff.py`, `reference_generators.py` (src/tejen_mission/tejen_mission/)
- `transfer_backend_node.cpp` (src/tejen_dynamic_planner/)
- `partner_mission.launch.py` (src/tejen_mission/launch/)
- `irl_commissioning.yaml`
- `CHANGES_FROM_TEJEN.md`
- `rig_fm9h.sh` (the rig line and its RET switch)
- The replay harness `results/analysis/cpp_realtime_2026-11-13/full/` and its README
- The lab chat's exit readings of 13 Nov (`exit_readings/`: scripts and their output)
- The twin rows R4404-R4441 (registry extract)
