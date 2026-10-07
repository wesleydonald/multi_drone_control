# drop_and_land (8 Oct 2026): a second loss or a capsizing ring drops the ring and lands the drones

## 1. Question

Wesley, 7-8 Oct: "it is possible for additional drones to detach by mistake, so I am hoping for a
mechanism that can detect (a completely unknown attachment (or two)), release the magnets and land
the other drones safely." On the ring tilt limit: "I am okay with 30 degrees". It applies to any loss
that leaves fewer than 3 drones on the ring.

## 2. Hypothesis

Today, in detach mode (`dissipative_planner`, `reconfig_mode ocp`):
- the first unannounced loss is detected and the fleet resizes to 3 (U1: R1140, R1145);
- a second loss is detected, but `_detach_ocp` refuses it (`min_survivors` 3). The planner keeps
  planning three cables, one of them gone.
- The ring then tilts until the trackers' 60 deg payload-tilt fault fires. That fault disarms the
  whole fleet in the air (the locked "fleet-wide disarm on any envelope fault"), and every drone
  falls about 1.5 m with its rod.
- On the rig on 7 Oct, "once 2 drones detached everything pretty much disarmed".

With drop-and-land:
- the refused second loss, or a ring tilt above 30 deg for 2 fresh ticks while airborne, switches
  every remaining magnet OFF;
- every drone becomes a departed drone: it holds, steps out to 1.2 m from the ring centre, and
  descends on its own touchdown detector;
- the OCP stops;
- a latched `/fleet/payload_dropped` stands down the trackers' payload checks, so the falling ring
  cannot trip the 60 deg fault;
- `/fleet/landed` follows once every drone is down, and the fleet manager disarms then.

Expected: no drone is disarmed above 0.2 m, and every drone lands within 15 s of the trigger.

## 3. The one variable

The drop-and-land path. New code, behind one parameter `drop_on_loss` (default false on this branch
until the twin verdict; true in the twin arm configs):

| piece | where | what |
|---|---|---|
| trigger A: loss below 3 | `dissipative_node._detect_tick` | a detection that `_detach_ocp` refuses for `min_survivors` -> `_drop_and_land('second loss: ...')` instead of `_detect_refused` |
| trigger B: ring tilt | `dissipative_node` planner tick | ring tilt > `drop_tilt_deg` (30) on 2 consecutive fresh ring poses (< 0.1 s old), airborne (load z > rest + 0.15 m), magnets present (`detach_magnet` or sim joints) |
| action | `_drop_and_land` | magnets OFF (rig) / joint detach (twin) for every attached drone; each becomes departed with a hold at its pose; phase `dropped` (no OCP solve); LAND-style clear-and-descend for all; latched `/fleet/payload_dropped` (Bool, transient local) |
| trackers | `tracker_node` | on `/fleet/payload_dropped`: payload tilt/staleness checks off; the drone envelope (own tilt, speed, ref age) stays |
| fleet manager | `fleet_manager_node` | on `/fleet/payload_dropped`: announce it; on `/fleet/landed` disarm as after a LAND |

An announced `/fleet/detach` that would leave fewer than 3 is still refused: nothing has failed, so
nothing is dropped.

## 4. Baseline

| run | what | numbers |
|---|---|---|
| R1140, R1145 | U1: one unannounced release on 1/3/5/9, detection on | detected 0.31-0.38 s; tilt peak 7.5-7.9 deg; landed |
| new arm B0 (week4 code) | two unannounced releases (drone 2 at t 20, then drone 1 at t 26), 1/3/5/9 hover | expected: second detection refused, ring capsizes, fleet disarmed in the air |

## 5. Pass / fail numbers

Twin, from the trigger to 15 s later.

| metric | supports | falsifies |
|---|---|---|
| any drone disarmed above 0.2 m | none | any |
| magnets OFF after the trigger | every attached drone within 0.2 s | later or any left ON |
| drone tilt after the trigger | <= 30 deg | > 45 deg |
| drone climb after release (peak z over the trigger z) | <= 0.3 m | > 0.5 m |
| closest pair of drones while landing | >= 0.6 m | < 0.4 m |
| every drone down | within 15 s, `/fleet/landed` then disarm | otherwise |
| landing spot from the ring's rest centre | >= 1.0 m | < 0.7 m (on the rod) |
| one release only (arm D3) | no drop; behaves as U1 (R1140/R1145) | any drop |
| replay over the 7 Oct rig logs (ring tilt, 2 fresh ticks over 30 deg, airborne) | fires only where the ring really went over (r0013 capsize; the second-loss run) | any other fire |

## 6. Repeats

Two per arm in the twin (Gazebo):

| arm | events | config |
|---|---|---|
| B0 baseline (week4) | release drone 2, then drone 1 | `w4_3915_release2.yaml` on week4 |
| D1 drop on a second loss | the same | `w4_3915_release2_drop.yaml` |
| D2 forced tilt trigger | hover; `drop_tilt_deg 1.0` (hover tilt 1.5-2 deg) | `w4_3915_drop_tilt.yaml` |
| D3 one release, drop armed | U1's events | `w4_3915_release_drop.yaml` |

## 7. Cost

8 Gazebo runs (B0 x2, D1 x2, D2 x2, D3 x2), lab PC one slot plus the laptop, ~40 min.

## 8. New code?

Yes, as in section 3, with unit tests:
- trigger A fires only when refused for `min_survivors`;
- trigger B needs two fresh ticks, airborne and magnets;
- every magnet is OFF;
- the dropped phase never solves;
- the trackers ignore payload faults once dropped;
- `/fleet/landed` follows only when every drone is down.

It lives on branch `safety-defaults` and is not flown on the rig until the twin passes and Wesley
says so. The CLAUDE.md lock "fleet-wide disarm on any envelope fault" is amended on this branch:
in detach mode, these two triggers drop and land instead; the 60 deg envelope stays as the last
resort. If falsified, the code is recorded in the registry and deleted.

## Critic

Critic agent, 8 Oct (read-only on code; pasted here by the main session), condensed.
Verdict: **rewrite the card**. As specified, neither trigger reaches the trackers before their 60 deg
payload fault on the events it is meant for:
- on r0013, trigger B fires at or after the fault;
- trigger A's detection (0.31-0.38 s) takes about as long as a ring on two cables needs to pass
  60 deg.

The design would not have saved either 7 Oct rig event, and the twin bars cannot show this.

**The five questions**
1. **Tried before?** Not as a whole. `docs/design/fleet_safety.md` §1 (2026-08-04) chose no recovery
   manoeuvre: "a safety path with no manoeuvre in it cannot have a bug in the manoeuvre". Its parts:
   - single-loss detection: R1139-R1145, SUPPORTS;
   - freed-drone step-out and descent on LAND: R1009-R1034, 18/18;
   - `departed_land` in flight: R1149/R1150 since.
2. **Simplest explanation it does not address: second losses cascade.**
   - After the first slip in r200006, the gap's two neighbours felt about twice their pull (10.5
     against 5.5 m/s²).
   - The second loss came 2.6 s into the operator's LAND.
   - On 1/3/5/9, a random first loss (plate 1, 5 or 9 in the old numbering) already leaves the
     centre of mass on or outside the survivors' polygon.
3. **What falsifies it, and do the bars test it?** It is falsified if any tracker latches a fault
   before the stand-down arrives. Only D1 tests this, and only in the twin:
   - the twin runs at real-time factor 0.3-0.45, so message delays look short against the physics;
   - it cannot capsize at r0013 settings (R1125);
   - the replay bar has no timing;
   - three bars pass by construction: magnets OFF within 0.2 s, landing spot >= 1.0 m, and D3 on
     1/3/5/9.
4. **What it breaks if it works:**
   - the locked fleet-wide disarm (needs Wesley's line in decisions.md and CLAUDE.md);
   - research code joins the safety path;
   - the latched topic outlives the flight under re-ARM without relaunch (618e315);
   - attach/m2 modes reach 39-47 deg (R0196/R0197, R0234), so the 30 deg trigger must stay off
     there.
5. **Asked for?** Yes (Wesley 7-8 Oct; 30 deg is his). But r200006, the one real accidental second
   loss, was in carry mode during LAND.

| event | r0013 (announced detach, 1/3/6/9) | r200006 (second loss, carry mode, in LAND) | twin second loss (estimate) |
|---|---|---|---|
| ring passes 30 deg | +0.33 s after magnet OFF | before the 41.1 deg warning at 858.588 | ~+0.2 s |
| trackers' 60 deg latch | +0.47 s (934.636 -> 935.107) | 858.755, ring at 0.24 m | ~+0.3 s |
| trigger B (2 fresh 10 Hz ticks) | +0.43 at the earliest | none (carry mode) | - |
| trigger A | none (3 survivors) | none (`_detect_tick` returns in LAND) | +0.31-0.38 (R1140/R1145) |

**Must-fix**
1. **Fly B0 alone first and read the timeline.**
   - If the latch comes first, no message from a 10 Hz planner can win.
   - Measure the rig's magnet command-to-release delay (r0012/r0013).
   - If the fault wins, the decision starts where the evidence arrives first: with `drop_on_loss`
     on, each tracker's payload-tilt fault means "hold and ask for the drop", not disarm.
2. **Trigger B's filter.** Persistence trades against r0013's 0.14 s window and does not reject a
   mis-fit pose. Filter by physics instead:
   - check every ring sample;
   - reset on an attitude step no ring can make between frames;
   - require the attached cables to stay within rod + 6 cm under the tilted pose;
   - use a tilt-plus-rate threshold sized on the replay.
3. **Replay with timing.** Firing after the latch (935.107, 858.755) is a fail. Replay every rig day
   with ring poses, and report the largest tilt outside a capsize.
4. **A random slip, not the designed leaver.**
   - Drop at detection when the survivors leave a gap >= 180 deg (`_largest_gap_deg`).
   - Add a release of the plate-9 drone (old numbering).
   - Fly D3 on 1/3/6/9.
5. **D2 fires in the lift ramp** (1 deg against rest + 0.15 m). Force the drop by a runner event
   after the lift.
6. **Latch lifecycle.** Clear `/fleet/payload_dropped` at ARM (re-ARM without relaunch); unit test.
7. **LAND and scope.**
   - A and B do nothing in LAND or during an unload.
   - r200006 was carry mode in LAND.
   - Make the triggers work in LAND; state that carry flights keep the disarm, or put the triggers
     in the base planner.
   - Match B's airborne gate to the trackers' check (no height gate; r200006 disarmed at 0.24 m).

**Should-fix**
- **Commands during a drop:** LAND (the operator's reflex), DETACH, ATTACH, TAKEOFF; a drop during an
  unload; a detection during the creep.
- **`/fleet/landed` with every drone departed:**
  - guard the vacuous "all survivors down";
  - gate on `_departed_down`;
  - the fleet manager must set `landing`.
- **Measured pull after the release:** on cable_source mocap, node 0 flies reference + (measured -
  reference), and the applied pull slews at 25 m/s² per s (0.2-0.5 s). r200006: drone 2 went +0.14 m
  up, 0.29 m out, then ~1 m/s outward. Add excursion and peak-speed bars; consider resetting the
  applied pull at the drop.
- **Holds after a capsize:**
  - seed the hold from the measured velocity (zero velocity is a hard brake);
  - latch the ring centre for the step-out at the trigger.
- **Re-send magnet OFF every tick until landed.**
- **Closest-pair bar:** the 1/3/5 drones start 0.5 m apart; define the window.
- **The landing-spot bar** is DEPARTED_CLEAR_R by construction: report it, not a falsifier.
- **Name drones by topic and plate.**
- **Neighbour arms:** a simultaneous double release, a second loss in LAND, a second loss in the
  circle.
- **Clocks:** freshness is wall clock while ticks are sim time; log when A and B go blind.
- **Refuse `drop_on_loss` outside detach mode.**

**Also raised**
- **Only the rig can test:**
  - the magnet release delay;
  - slack and snatch;
  - rod whip;
  - ring-pose glitches;
  - ground idle on a drone resting on its rod;
  - the ring taking 1 m drops.

  First rig step: a forced drop by topic from a low hover (ring at 0.3 m).
- **A twin win in the race says little about the rig** (RTF 0.3).
- **A cheaper way to cut exposure that keeps the ring:** LAND automatically after an unannounced
  detection.
- **The lock amendment** is Wesley's line in decisions.md and CLAUDE.md before any code.

## Revision v2 after the critic (8 Oct night)

**Design changes** (the critic's must-fix list):
1. **The trigger starts in the trackers** (must-fix 1).
   - Each tracker, on every ring sample, asks for the drop on `/fleet/drop` when the ring tilt is
     over `drop_tilt_deg` (30) for `drop_samples` (2) samples in a row.
   - A frame-to-frame attitude step over `drop_glitch_step_deg` (20) counts as a pose glitch: the
     sample is ignored and the count resets (must-fix 2).
   - The tracker that asks stands its own payload checks down at once. All four see the same ring,
     so the 60 deg latch cannot win the race.
   - If no `/fleet/payload_dropped` arrives within 0.5 s, the checks come back, and the 60 deg fault
     disarms the fleet as today.
   - The rig values of the rule come from the bag replay (`2026-10-08_drop_trigger_replay.md`,
     must-fix 3). The twin runs test the mechanism.
2. **The planner's triggers** (must-fix 4, 7):
   - a detected loss that leaves fewer than `min_survivors`, or survivors spanning a gap >= 180 deg
     (`_largest_gap_deg`);
   - any detected loss in LAND (detection now runs in LAND until the ring is down, with
     `drop_on_loss`);
   - a `/fleet/drop` request.

   An announced `/fleet/detach` that would leave too few is still refused.
3. **The action** (should-fix):
   - Every magnet OFF, re-sent every tick until landed.
   - The twin's joints are released once.
   - Every drone not already gone becomes departed:
     - its hold is seeded 0.2 s ahead on its velocity;
     - it steps to 1.2 m from the ring centre latched at the trigger;
     - it lands on its own touchdown detector.
   - A drone that left earlier keeps its plan and lands too.
   - The OCP stops.
4. **Latch lifecycle** (must-fix 6). `/fleet/payload_dropped` is latched (transient local). The
   planner publishes False at start and at every ARM before a drop; the trackers clear their
   request on the re-ARM edge.
5. **Commands in a drop** (should-fix):
   - LAND is acknowledged (re-announces `/fleet/landed` once down);
   - DETACH is ignored;
   - the fleet manager enters landing on `/fleet/payload_dropped` and disarms on `/fleet/landed`;
   - `/fleet/landed` comes when every drone is down, or after 30 s.
6. **Scope** (must-fix 7):
   - detach mode only; `drop_on_loss` is refused unless `reconfig_mode ocp`;
   - carry flights (mode mpc) keep today's fleet-wide disarm;
   - the 60 deg envelope stays as the last resort.
7. **Not adopted:** resetting the trackers' measured pull at the drop. The measured term decays with
   the real pull; zeroing it before the magnet lets go would pull the drone toward the ring for the
   release delay.

**Arms (twin, branch safety-defaults, 2 runs each), replacing section 6:**

| arm | events | config | expected |
|---|---|---|---|
| B0 baseline (week4) | release index 1 (90 deg), then index 0 (30 deg) 6 s later | `w4_3915_release2.yaml` | today: capsize, fleet disarm in the air |
| D1 second loss | the same, `drop_on_loss` | `w4_3915_release2_drop.yaml` | drop at the second loss (A, or the trackers' tilt rule) |
| D2 forced | steady hover, a DROP event after the lift | `w4_3915_drop_forced.yaml` | drop from 1.0 m |
| D3 one release, a hard layout | U3 on 1/3/6/9 | `w4_3969_release_drop.yaml` | NO drop (peak 13.9-19.2 deg < 30) |
| D4 the bad leaver | release index 3 (270 deg): survivors 30/90/150, gap 240 | `w4_3915_release3_drop.yaml` | drop at detection (gap rule) |
| D5 double loss | indices 1 and 0 together | `w4_3915_release01_drop.yaml` | drop |

**Bars (section 5, revised):**
- **Must hold:**
  - no drone disarmed above 0.2 m;
  - no tracker fault latch before the stand-down;
  - drone tilt <= 30 deg after the trigger;
  - climb <= 0.3 m;
  - horizontal excursion and peak speed reported (speed fault 3 m/s);
  - every drone down within 15 s and `/fleet/landed` then disarm;
  - D3 never drops.
- **Reported, not falsifiers:**
  - the landing spot (DEPARTED_CLEAR_R by construction);
  - the closest pair, from the trigger to touchdown.
