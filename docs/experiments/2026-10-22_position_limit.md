# Card: the position limit (22 Oct; Wesley's push answer 4)

Wesley (22 Oct): any quad's x or y beyond +-1.75 m from the mocap and world origin makes the fleet LAND (an envelope
LAND, not a disarm), and nothing is commanded or landed outside it. This replaces the "no geofence" of 4 Aug
(`utility_objects/safety.py`). Built behind `pos_limit_m`, off in the code and in every profile until this card's
critic. Uncommitted, in the main tree with the 1 m defaults. Revised the same evening after the critic (below):
the freed quad's landing point (M1, m1), the rig-log replay (LIM-0) and the runs; the answers are under "Response".

## 1. Question
Does a +-1.75 m limit land the fleet before a quad leaves the mocap volume, without landing a legitimate flight and
without changing any flight that stays inside it?

## 2. Design as built

**The LAND (fleet manager).**
- It subscribes every managed drone's `/drone_<i>/motion_capture_state` when `pos_limit_m` > 0. That includes QUAD4
  in the attach graph, also while Tejen's stream flies it.
- It counts a breach when max(|x|, |y|) > `pos_limit_m` while the fleet flies (TAKEOFF to disarm) and the drone is
  airborne: z above min(its height at ARM + 0.10, 0.25) m.
- After `POS_LIMIT_SAMPLES` (3) poses in a row it publishes one LAND on `/fleet/command` through `_fault_land`. That
  is one LAND a flight, shared with the tilt cut and his stream's disarm. The planner then descends and every drone
  lands; the manager disarms at `/fleet/landed`.
- The reason goes on the latched status line, once per excursion:
  `POSITION LIMIT: drone k at x .., y .. m, past +-1.75 m: LAND, every drone lands.`
- It never sends the emergency stop and never disarms.

**Nothing commanded outside (planner and trackers).**
- `LoadPlanner._publish_ref` is the one publisher of every planner reference: the OCP, the creep, the LAND hold, the
  departed drones, the newcomer's approach, the take-back and the network mode. It clamps each node's x and y to
  +-(limit - 0.05) = 1.70 m.
- A velocity component that carries a clamped node further out is zeroed, so the drone is asked to stop at the
  clamp. The thrust and pull feed-forward are left as they are.
- The trackers apply the same clamp on receipt (`_planner_ref_callback`), so free_hover and any other sender are
  covered. Each clamp logs a throttled warning.

**Nothing landed outside (detach mode).** Each landing point is kept inside +-(limit - 0.10) = 1.65 m. A step out
from the ring is shortened along its own ray from the ring's centre, so it stays radial (`limit_ray`; critic m1). Where
that leaves a freed quad inside its clearance circle (1.2 m about the ring's measured centre, or its step's own length
when shorter), it goes instead to the point of that circle inside 1.65 m whose bearing is nearest its own
(`clear_point`; critic M1, the R0567 tip-over). It is declared clear, and descends, only there. When no point of the
circle is inside, it holds where it is, at height, and the status line says `drone k HELD: no landing spot clear of
the ring inside the limit`. That applies to:
- the LAND step-out;
- the OCP-resize step-out (with `departed_land` it lands from there);
- the path-clear point (around the path's centre; none inside: it lands from its ring step);
- a drop's step-out (a quad already past 1.65 m comes in, still clear of the ring);
- an aborted newcomer's step-out.

Radial only (no clearance circle): the take-back's out point and its failed-guard step beside plate 9 (in zone c, a
moving ring, the out point is clamped where it is latched and then moves with the ring, so only the 1.70 m reference
clamp bounds it; his ball's clearance is 3c's open ruling). x and y clamped apart: the network mode's detached
landing (legacy only).

**Knobs.**
- `pos_limit_m` (m; 0 = off) on the fleet manager, the trackers and the planner (detach mode included).
- The profile holds `pos_limit_m: ''` in `common.yaml`. It is a params_file knob, not a launch argument. The twin's
  LIM runs type it through `configs/twin/pos_limit*.yaml` (their base params file plus `pos_limit_m`). How the rig
  types it is M2, Wesley's.
- The margins and the sample count live in `utility_objects/position_limit.py`.

**Tests (offline, all pass).**
- `src/utility_objects/test/test_position_limit.py` (the helpers);
- `src/fleet_manager/test/test_position_limit_manager.py` (LAND once; floor, inside, off and one bad pose ignored;
  reset at ARM);
- `src/mpc_planner/test/test_position_limit_refs.py` and `src/tracker/test/test_position_limit_tracker.py` (clamp
  on and off);
- `src/dissipative_planner/test/test_one_metre_clearances.py` (LAND step-out and drop near the limit land inside it
  and clear of the ring; the lobe-tip release lands on the clearance circle at the nearest bearing; no point inside:
  held at height, on the status line; off changes nothing);
- `tools/test/test_limit_replay.py` (LIM-0's replay);
- `src/bringup/test/test_real_mode_launches.py::test_the_position_limit_passes_nothing_until_set_then_reaches_every_node`.

## 3. Failure modes it must not cause (for the critic)

**a. A LAND at a legitimate figure-8 lobe tip.** At size 0.9 with 1 m cables the outboard carrier peaks nominally at
1.625 m: 7.5 cm inside the reference clamp and 12.5 cm inside the LAND line. These eat into that margin:
- the ring's start offset: the paths are relative to where the ring lifts, the limit to the mocap origin, and every
  22 Oct flight lifted 2-5 cm toward -x;
- tracking error and overshoot: rig f3h had a 3-D RMSE of 4.0 cm (size 1.0, 0.55 m cables);
- the cable's lean on the turn: about 1 deg at 0.2 m/s, about 2 cm out at 1 m;
- the pair swing on the rig: 2.6-3.7 deg in the old-filter climbs, about 5-6 cm at 1 m.

Worst case, the margin to the LAND line is a few cm. The size-1.0 figure-8 (the twin's tables) reaches 1.725 m
nominally. That is past the reference clamp on every lap, so the formation deforms and a 2.5 cm overshoot lands it.

**b. A freed quad's step-out.**
- A hover release at the origin steps out to 1.2 m: inside the landing margin, unchanged.
- A release at a lobe tip (ring at x 0.9) would step radially out to 2.1 m. Clamped to 1.65 m as first built, it
  landed about 0.75 m from where the ring was, inside the 1.2 m clearance (critic M1). Since the revision it goes
  round the clearance circle to where it meets 1.65 m (51 deg from the ring's x axis for a quad at 30 deg).
- A quad that lands where it is, between 1.65 and 1.70 m (no step-out), is held by the 1.70 reference clamp only.
  That is a known gap.
- On the twin's rigid rods, LAND's unwind moves the carriers out by up to L(1 - cos 60) = 0.5 m. Near the limit, the
  clamp then holds an unwinding carrier at 1.70 m while its rod holds it up. This is twin only: the rig flies
  `land_unwind` false.

**c. His QUAD4 approach.** While Tejen's stream flies QUAD4, our clamps do not touch his references; only the LAND
applies.
- QUAD4's mat and his ball's spot must be inside +-1.75 m, with margin, or the fleet lands as soon as he lifts QUAD4
  there.
- The 1 m twin pickup worlds put his ball at (1.6, 2.0) (`make_attach_worlds_L100.BALL_XY`). With the limit on,
  every 1 m basket, floor-join and full-M1 twin run would land on his pickup.
- On a LAND during his flight under `partner_contact`, our take-back flies QUAD4 out (now clamped to 1.65 m). The
  critic should walk that sequence: the LAND, `LAND_TAKE_BACK_SETTLE_S`, the take-back, his planner's LAND.
- With his ball on, the take-back clears to `DEPARTED_CLEAR_R` + body-to-tip, which is 1.2 + 1.04 = 2.24 m from the
  ring at 1 m. That is past the limit even from a ring at the origin. The clamp brings QUAD4 down at 1.65 m, so the
  ball lands about 0.6 m inside the clearance its rod needs: closer to the ring and the carriers than the take-back
  was designed for (R1482: with the ball, QUAD4 landed 0.33 m from a carrier). This needs a ruling before the limit
  flies with his ball: a shorter clearance at 1 m, or no limit during his flight. The 1 m twin take-back check,
  `m1c_land_transit_L100` (limit off), flies the unclamped case first.

**d. Others to rule on.**
- A start outside: a drone resting beyond 1.75 m, or a ring placed off centre, lands the fleet at lift-off.
- Mocap: 3 samples is 25-35 ms at 88-120 Hz. A mis-routed body (another body's pose) that stays out lands the
  fleet. A LAND, not a disarm.
- The clamp deforms the formation: a path that asks a carrier past 1.70 m holds that carrier while the others move
  on. The cable geometry and the tension split change (a rigid twin rod tilts the ring; a slack rig cable goes
  slacker). The LAND follows only if the drone itself reaches 1.75 m.

## 4. What trips it today (nominal: 1 m cables, 60 deg, no lean, error or swing; the ring starting at the origin)

| Reach (m) | Configs | With the limit on |
|---|---|---|
| 1.725 | `traj3_L100_linex_v{125,200,300}` (line_x, 1 m out) | past the 1.70 m clamp, 2.5 cm from LAND |
| 1.725 | `b{2,3,4}_fig8big_h5_v200_L100_{ema,slope}` and `_cd060` (figure-8 size 1.0) | past the clamp, 2.5 cm from LAND |
| 1.725 | `b2/b4_linespinc_h5_v200_L100_{ema,slope}` (and `_cd060`), `b2/b4_linespin_v200_L100` (line_spin 1 m out) | past the clamp, 2.5 cm from LAND |
| 1.685 | `traj3_L100_spin_v{125,200,300}` (spin r 0.5) | inside the clamp, 6.5 cm from LAND |
| 1.725 | the M1 layout with QUAD4 as fourth carrier on an orbit r 0.5 (plates 3/11/7/9) | past the clamp |
| - | his ball at (1.6, 2.0) in the 1 m pickup worlds: `basket_*_L100`, `m1c_*_L100`, `m1_full_*_L100` | LAND on his pickup |

The Wed 28 sheet (`docs/rig/wed_2026-10-28.md`):

| Sheet line | Reach (m) | With the limit on |
|---|---|---|
| ls2B (two quads, spinny line 1 m out) | 1.711 | past the clamp, 3.9 cm from LAND |
| three-quad spinny line | 1.645 | inside |
| f3E, f4B (figure-8 size 0.9) | 1.625 | inside |
| f2B (figure-8 size 0.9) | 1.528 | inside |

Scan: `scratchpad/wave1/limit_scan.py` (read-only).

Wesley, 22 Oct about 22:05: the rig's figure-8 shrinks to size 0.75 and the line paths to 0.75 m out, so the outboard
quad's nominal reach on 1 m cables is 1.475 m (22.5 cm inside the clamp, 27.5 cm inside the LAND line). The defaults
(`fig8_size`, the line distance) change with his word; the tables above are the 1.0 and 0.9 m paths as they were.

## 5. To decide (Wesley, the critic)
- A mark for the ring's start at the origin, or a check that the ring starts within a few cm of it.
- The 1 m line_x and line_spin distance (1.0 now, which reaches 1.725 m). Shorten them, or keep the limit off for
  them.
- Whether the twin's size-1.0 tables fly with the limit off.
- His mat's and his ball's positions inside the limit (rig and twin).
- 3 samples, or fewer with a speed term (an early LAND for a runaway).
- Whether a quad landing where it is between 1.65 and 1.70 m needs its own step in.

## 6. Runs (rewritten 22 Oct after the critic)

Bars are PROPOSED: the integrator freezes them (or changes them) before the first run, and runs already seen are never
re-scored. Every twin run is exploratory until a claim is made; 2 repeats in Gazebo. The configs pass
`tools/run_experiment.py <cfg> --check` (the load-inertia warnings are their bases').

**LIM-0: the rig-log replay (done offline; critic M4).** `tools/limit_replay.py` feeds every 21-22 Oct rig flight's
quad and ring poses (`<flight>_run/logs/run.csv`, the bag on a 50 Hz grid) through the fleet manager's own test
(`breach`, 3 samples in a row, airborne, TAKEOFF to DISARM). Each quad's horizontal offset from the ring is scaled from
the flight's reach (0.5 m: 0.55 m cables at 60 deg, from its params.json) to the 1 m formation's 0.725 m, up to the
flight's first release (after it, as flown). `--path-scale 0.75` also shrinks the ring's path about its hover point
(its tracking error kept): the 0.75 m paths of Wesley's 22 Oct decision. The table is under Response, M4.

**LIM-e: the forced breach (the untested half of the critic's falsifier).**
`lim_e_b3_fig8s075_h5_v200_L100_ema_lim120`: 3 quads, the size-0.75 figure-8, `pos_limit_m` 1.2
(`configs/twin/pos_limit120_drop_and_land_hold5.yaml`), so the outboard quad's nominal 1.475 m is past the line.
PROPOSED pass, all of:
- exactly one LAND (one POSITION LIMIT status line, one `/fleet/command` LAND from the manager), within 3 mocap
  samples of the first pose past 1.2 m (scored from run.csv at 30 Hz: the LAND no later than 0.1 s after its first
  sample past 1.2 m);
- no disarm or abort before `/fleet/landed`;
- every landing point inside L - 0.10 = 1.10 m on both axes; a quad that is freed or dropped also 1.17 m or more from
  the ring's centre;
- no drone tilt over 30 deg at touchdown.

Expected to miss as built, for the integrator to decide when freezing: the carriers land with the ring, held only by
the reference clamp at L - 0.05 = 1.15 m (no step in for a carrier: section 3b's known gap), and the twin's LAND
unwind (`land_unwind` true, rigid rods) pushes them out against it. Either the bar for a carrier is L - 0.05, or the
design gets a carrier step-in first. The reference clamp at 1.15 m also deforms the formation before the breach
(3d): that run shows by how much.

**LIM-a: nothing changes inside the limit (falsifier (ii); critic M3, M4).** The size-0.75 figure-8, the limit on at
1.75 m (`configs/twin/pos_limit_drop_and_land_hold5.yaml`), ema, undelayed: `lim_a_b2_fig8s075_h5_v200_L100_ema`,
`lim_a_b3_...`, `lim_a_b4_...`; and one delayed arm, `lim_a_b2_fig8s075_h5_v200_L100_ema_cd060` (60 ms of command
delay from 26.5 s sim, after the climb: the twin that reproduces the rig's pair swing; the pair, because the rig's
pair is the one that swings and f2slope is LIM-0's closest flight). PROPOSED pass: no LAND, no position-limit clamp
warning from the planner or a tracker, no abort; report each quad's largest |x|, |y| and its margin to 1.70 and 1.75.
With no clamp warning the references are the limit-off ones (the clamp acts only past 1.70 m), so no limit-off arm
is needed.

**LIM-c: a freed quad at the lobe tip (critic M1, m1).** `lim_c_c4_detach0tip_fig8s075_L100` (limit on,
`configs/twin/pos_limit_chain_pair_noxy.yaml`) against `lim_c_c4_detach0tip_fig8s075_L100_off` (the same, limit off),
2 repeats each. Both are `c4_detach9_fig8_L100` with the figure-8 at size 0.75 and the hard DETACH moved from plate 9
to plate 0 at 22.5 s: plate 9 never meets the limit on this path (its step-out peaks at y 1.6 m), plate 0 is the
outboard quad at the +x lobe tip (R2311's path started 14.6 s config time; the tip is 7.7 s into the size-0.75 lap).
Its 0.5 m step-out (or a drop's 1.2 m, as R2311 dropped) is past 1.65 m, so the clearance circle decides its landing
point. PROPOSED pass, every freed or dropped quad:
- lands inside 1.65 m on both axes (on arm only);
- starts its descent and touches down 1.17 m or more from the ring's centre at that moment, and from the ring's rest
  point (its 1 m tether cannot lie under the ring);
- tilts under 30 deg at touchdown, and no HELD on the status line (at 1.75 m every circle has a point inside).
The off arm shows the radial landing the clamp replaces (about 1.2 m out, past 1.75 m).

**Bars FROZEN 22 Oct 22:25, before any LIM-a/c/e run (the integrator):** as proposed above, with one change to
LIM-e's landing bar, made before any run because the design has no carrier step-in: a CARRIER passes if its reference
never goes past L - 0.05 (1.15 m) and it touches down inside 1.17 m on both axes (the clamp plus 2 cm of tracking); a
FREED or DROPPED quad passes inside L - 0.10 + 0.02 = 1.12 m and 1.17 m or more from the ring's centre. Repeats: LIM-e
x2, LIM-c on and off x2 each, LIM-a x1 per fleet size (a check that nothing changes; a LAND or a clamp warning in it
is a fail, not a reason to repeat). Runs already seen are never re-scored against other bars.

Dropped: LIM-b (the 1 m spinny line: Wesley's 0.75 m lines replace it; a config that reaches the clamp is expected to
LAND, M3). Deferred: LIM-d (his ball inside the limit) until 3c's ruling on his ball and mat.

## Critic
*The project's critic, 22 Oct 2026 (read-only review of the card and the code as built, before any run).*

**1. Has it been tried?** Yes. A position and altitude bound was built and then removed on Wesley's word on 4 Aug (`src/utility_objects/utility_objects/safety.py:14-20`, `docs/design/fleet_safety.md:23`), because "a position check catches nothing a ready human does not". No registry row has flown it since. This card turns it into a LAND instead of a disarm, so it is not a re-run.

**2. Simplest explanation it misses.** The only time a quad has run out on the rig is RIG-1014-t5_2. A released quad ran outward at 1.4 m/s on its carrying thrust, reached 1.57 m, and then the fleet disarm cut it. A LAND only changes the reference, so it cannot stop a quad that has stopped following its reference. The fault disarm and `drop_on_loss` already cover that case. A quad that is tracking only gets past 1.75 m if it is commanded there, and the 1.70 m clamp already stops that. So the LAND will mostly fire on ordinary tracking error.

**3. What would falsify it, and do the runs test it?** It is false if either of these happens:
- (i) A forced breach does not give exactly one LAND, with no disarm and every landing point inside 1.65 m.
- (ii) A flight that is nominally inside trips it.

LIM-a to LIM-d only test (ii). They also fly the undelayed ema twin, which does not reproduce the rig's 1.4 Hz pair swing (that needs about 50-60 ms of added delay, per SESSION_STATE). Nothing tests (i), or the card's own question ("before a quad leaves the mocap volume").

**4. What it breaks if it works.**
- Wed line ls2B (the card says 1.711 m; the sheet says about 1.725 m).
- Every 1 m line_x and line_spin, and the size-1.0 twin tables.
- Every 1 m pickup world, where his ball sits at (1.6, 2.0).
- The freed quad's clearance (R0567, R1482).

The yaw-only network is unaffected: the clamp acts on the reference only, with no feedback from the load pose. The gate is unaffected while the limit is off.

**5. Was it asked for?** Yes: `docs/decisions.md:396-397` (push answer 4). Wesley did not ask for the rule that a clamped step-out "counts as clear", nor for the start mark.

### Findings
There is no blocker while the limit ships off. The first three below need fixing before LIM-c or before it flies on the rig.

**Major**
- **M1: a freed quad lands across its own tether. This can crash a drone.**
  - Where: `dissipative_node.py:1039-1042`. When the drone is within 2 cm of the clamped goal, it sets `_departed_clear` and descends, even though it is still inside the clearance radius.
  - Scenario: a release at a lobe tip, with the ring at x 0.9, lands the quad about 0.75 m from the ring centre with 1 m of tether trailing back toward the ring.
  - Precedent: that is the R0567 layout (0.63 m out, the ring pinned the cable and tipped the drone at touchdown). `c4_detach9_fig8_L100` already flies this case.
  - Run change: LIM-c must also score the freed quad's tilt at touchdown and whether the tether is under the ring's landing spot, not only "landing point ≤1.65 m".
- **M2: the rig cannot get the knob onto any Wed line as planned (hardware path).**
  - A launch takes one `params_file`, with no includes (`src/bringup/bringup/profiles.py:73-75, 216-217`).
  - `pos_limit_m` cannot be typed on the command line (`profiles.py:208-211`).
  - Every Wed path already types `traj_hold.yaml`, M1 types `drop_takes_back.yaml`, and the pairs type `chain_pair*.yaml`. A separate `configs/rig/pos_limit.yaml` cannot be added to any of them.
  - The knob has to go into each of those rig files, or into `real.yaml`. `real.yaml` is a launch default, so that needs Wesley's word.
- **M3: only 5 cm between the clamp and the LAND line.**
  - `limit_horizon` (`position_limit.py:27-41`) stops the reference dead at 1.70 m. It zeroes the outward velocity but keeps the thrust feed-forward.
  - With the rig's path lag of about 0.19 s (RIG-1022-f3c) at 0.2-0.3 m/s, plus 5-6 cm of pair swing, any config that reaches the clamp should be expected to LAND.
  - At line_x's end hold the reference is at rest, so the xy model integral (`int_k_xy`) is active there and winds up against the clamp.
  - Run change: score LIM-b as "LAND expected". Those configs should fly with the limit off or be shortened, not just "documented".
- **M4: the lobe-tip margin at size 0.9 is only nominal.** The 12.5 cm margin has to absorb:
  - the ring's start offset (2-5 cm);
  - the pair swing (5-6 cm);
  - the peak tracking error, not the RMSE (roughly 2 × 4 cm).

  That leaves nothing. A cheap check needs no flight: replay the quad poses from every 21-22 Oct rig log (`results/rig/2026-10-22/`) through `_limit_check`, with the reach scaled to 1 m. That gives rig evidence directly.

**Minor**
- **m1: the step-out bends sideways.** `limit_xy` (`position_limit.py:19-24`) clamps x and y separately, which turns a radial step-out sideways. A quad at (1.53, 0.36) off a ring at (0.9, 0) heads off at 63 deg instead of 30 deg. Shortening the step along its own direction keeps it radial.
- **m2: after any LAND or drop, the limit goes quiet.** `_fault_land` returns early once a LAND or drop is under way (`fleet_manager_node.py:600`). A take-back in zone c, bounded only by the 1.70 m clamp, can then sit 5 cm from the line with no check. Say that this is accepted.
- **m3: QUAD4 can land outside under Tejen's LAND.** Without `partner_contact`, his planner lands QUAD4 straight down wherever it crossed the line, which is outside. "Nothing landed outside" cannot hold for his stream; Wesley needs to rule on it.
- **m4: the reach of ls2B disagrees** (card 1.711 m, sheet about 1.725 m). Reconcile before the sheet types it.

### Runs that would expose them
| Run | What it does | Exposes |
|---|---|---|
| LIM-0 | Offline replay of the rig logs (described under M4) | M4 |
| LIM-e | Forced breach: `pos_limit_m` 1.2 on the size-0.9 figure-8. Pass: one LAND within 3 samples, no disarm, every landing point ≤1.65 m | the untested part (i) |
| LIM-a | Also fly it on the twin with the 60 ms delay | M3, M4 |
| LIM-c | Add touchdown tilt and tether-under-ring checks, 2 repeats | M1, m1 |

Freeze the bars before any of these runs.

CRITIC: rewrite the card. It has no test that the limit actually fires (LIM-e) and no rig-log replay, and its clamped step-out brings back the R0567 tip-over. Fix M1 and M2 before LIM-c or any rig flight.

## Response (22 Oct)

| Finding | Status | What changed, or why |
|---|---|---|
| M1 a freed quad lands across its tether | fixed | The clamped step-out goes round the 1.2 m clearance circle about the ring's measured centre to the point inside 1.65 m nearest its own bearing, and is declared clear only there (`_land_clear`, `utility_objects.position_limit.clear_point`). The same for the OCP-resize step-out (`departed_land` lands from it), a drop's step-out, an aborted newcomer and the path-clear point. No point inside: it holds at height, HELD on the status line. LIM-c now scores touchdown tilt and the clearance (section 6). |
| M2 the rig cannot type the knob | needs Wesley | One params_file a launch, and `pos_limit_m` cannot be typed. Either `pos_limit_m: 1.75` goes into every rig file the Wed sheet types (`traj_hold*.yaml`, `drop_takes_back.yaml`, `chain_pair*.yaml`, `two_quad_*.yaml`), or into `real.yaml` (a launch default: his word). Recommended: `real.yaml`, one place that no line can forget; its cost is that the limit is then on for every rig flight, the hovers too. The twin's runs type it through `configs/twin/pos_limit*.yaml`. |
| M3 5 cm between the clamp and the LAND | accepted; answered mainly by the smaller paths | Wesley 22 Oct: figure-8 size 0.75 and lines 0.75 m out, so the outboard quad is nominally at 1.475 m on 1 m cables, 22.5 cm inside the clamp. LIM-0 with the paths at 0.75: no flight trips; the closest carried flight is the pair's f2slope at 1.655 m (4.5 cm inside the clamp), every other 1.57 m or less. A config that reaches the clamp (the twin's size-1.0 tables, the 1 m line_x and line_spin, a circle of r 0.5 at 1 m: nominal 1.725 m) is expected to LAND and flies with the limit off, as every profile has it. LIM-b is dropped. |
| M4 the lobe-tip margin is only nominal | answered by LIM-0 and the smaller paths | Table below. Two things the card had wrong: the path's start offset reached 16.4 cm (f2slope; f3h 7.6 cm, f3big 0.5 cm), not 2-5 cm; and the pair yawed so a quad lay along x at the lobe tip (f2slope, 0.50 m of 0.50 m reach along x), so the card's 1.528 m for f2B (30 deg off the axis) is no bound for a pair. The ring's start mark (section 5) is still worth a decision. |
| m1 the step-out bends sideways | fixed | Landing and step-out goals are shortened along their ray from the ring's centre (`limit_ray`): the critic's quad at (1.53, 0.36) keeps its 30 deg. The planner and tracker references keep the x and y clamp (`limit_xy`, `limit_horizon`). |
| m2 the limit goes quiet after a LAND or drop | accepted | One fault LAND a flight is by design (the tilt cut and his stream share it). After it the 1.70 m reference clamp and the 1.65 m landing points still bound every quad we fly; a take-back in zone c rides with the ring under the 1.70 m clamp only. |
| m3 QUAD4 can land outside under his own LAND | accepted (Wesley, 22 Oct) | Our clamps never touch his references; only our LAND applies to his stream. |
| m4 ls2B's reach | superseded | The sheet's 1.725 m is the full reach along x, which a spinning pair passes through; at 0.75 m out it is 1.475 m. Recompute when the sheet is retyped. |

Left open by the M1 fix, for the integrator and Wesley:
- A HELD quad is still disarmed in the air when the planner stops waiting for it: `/fleet/landed` goes out 15 s
  after the survivors touch down (`LAND_DEPARTED_WAIT_S`), 30 s after a drop (`DROP_LAND_WAIT_S`). It cannot happen at
  1.75 m with the ring inside 1.65 m: every clearance circle (1.2 m; 1.475 m for S's ring) about such a centre has a
  point inside. It needs a limit much tighter than the circle, or a ring outside the margin.
- With a tight limit the nearest inside point can lie across the ring (LIM-e's 1.2 m at a lobe tip: the inside arc
  starts at 114 deg), and the straight step there passes over the carriers. LIM-e has no freed quad; a drop in it
  would show this.

**LIM-0** (`tools/limit_replay.py`, exit 1: a trip; `--path-scale 0.75`, exit 0). Largest |x| or |y| of any quad
while airborne and flying (m); "1 m": offsets scaled from the 0.5 m reach to 0.725 m up to the first release. The
circles and spins are not shrunk by Wesley's decision: their x0.75 column is for reference.

| Flight | Path | Flown | 1 m | To LAND / clamp (1 m) | 1 m trips | 1 m, paths x0.75 |
|---|---|---|---|---|---|---|
| 21 c125_2 | circle r 0.5 | 1.469 | 1.662 | +0.088 / +0.038 | no | (1.412) |
| 21 c200_2 | circle r 0.5 | 1.460 | 1.652 | +0.098 / +0.048 | no | (1.403) |
| 21 c200_4 | circle r 0.5 | 1.378 | 1.589 | +0.161 / +0.111 | no | (1.339) |
| 21 c2q | circle, pair | 0.712 | 1.025 | +0.725 / +0.675 | no | (1.025) |
| 21 f125, f125_2, f200 | figure-8 0.5 | 0.917-1.029 | 1.146-1.269 | +0.481 or more | no | 1.023-1.144 |
| 21 l125 | line_x 1 m | 1.568 | 1.814 | -0.064 / -0.114 | d0 at 66.8 s | 1.566 |
| 21 l200 | line_x 1 m | 1.567 | 1.807 | -0.057 / -0.107 | d0 at 37.4 s | 1.557 |
| 21 s125 | spin r 0.5 | 1.393 | 1.610 | +0.140 / +0.090 | no | (1.373) |
| 21 s200_3 | spin r 0.5 | 1.426 | 1.644 | +0.106 / +0.056 | no | (1.412) |
| 22 f2slope | figure-8 1.0, pair | 1.678 | 1.903 | -0.153 / -0.203 | d1 at 48.7 s | 1.655 |
| 22 f3 | figure-8 0.5 | 0.973 | 1.199 | +0.551 / +0.501 | no | 1.075 |
| 22 f3big | figure-8 1.0 | 1.448 | 1.677 | +0.073 / +0.023 | no | 1.428 |
| 22 f3h | figure-8 1.0 | 1.435 | 1.670 | +0.080 / +0.030 | no | 1.420 |
| 22 ls3 | line_spin 1 m | 1.460 | 1.700 | +0.050 / 0.000 | no | 1.453 |
| 22 ls3c | line_spin 1 m | 1.425 | 1.633 | +0.117 / +0.067 | no | 1.403 |
| 22 hovers (h2m, h2rts, h2slope, h2t9p, h3, h3n, t3p_2) | hover | 0.694-0.892 | 1.004-1.208 | +0.542 or more | no | same |
| releases: 21 c200_3, c2q_noxy_3/_4, c2q_p11p5_2, s200, s200_2; 22 h2, t3p_3 | after the release: as flown | 1.065-1.645 | same | +0.105 or more | no | same |

No flight trips as flown at 0.55 m. At 1 m three would have landed the fleet, all on the 1 m paths Wesley has now
shrunk (the two line_x, the pair's size-1.0 figure-8). The releases' largest are freed quads after a drop (h2: both
quads climbed to 2 m and out to 1.645 m after its 18.8 s drop): with the limit on, those step-outs are now clamped and
kept clear of the ring. Skipped (no run.csv): c125, c200, c2q_noxy, c2q_noxy_2, c2q_noxy_5, c2q_p11p5, h2t9, t3p; l200_2
has no airborne pose (aborted at ARM).

## Results (22 Oct, scored against the bars frozen 22:25; registry R2435-R2444)

| Run | Arm | Verdict | Numbers |
|---|---|---|---|
| R2435 / R2436 / R2437 | LIM-a, 2 / 3 / 4 quads, size-0.75 figure-8, limit 1.75 | pass | no LAND, no clamp warning, no abort; largest reach 1.469 m (+0.231 to the clamp) |
| R2438 | LIM-a, 2 quads, 60 ms delayed twin | FAIL (abort) | not the limit: the pair swing grew (1.38 Hz, M1 7.07 deg) until a quad disarmed; reach 1.482 m |
| R2439 / R2440 | LIM-e, limit 1.2 m | FAIL (config error) | the quads start 1.225 m out: POSITION LIMIT fired once, 0.82 s after TAKEOFF, one LAND; drone 1 touched down at 1.226 m (> 1.17); the path breach was never tested |
| R2441 / R2442 | LIM-c on, a quad freed at the +x lobe tip | FAIL (clearance) | the detach became a full drop (ring past 45 deg in 0.5 s); freed quad inside the limit (1.630 / 1.640 m); dropped quads came down 0.86-1.14 m from where the ring landed; tilt <= 3.4 deg, nothing HELD |
| R2443 / R2444 | LIM-c off | (contrast) | freed quad at x 1.885 / 1.907 m; the same clearance misses |

Wesley 22 Oct ~23:20: the drop clearance does not matter on the rig (quads may land on the payload), and no 1 m push
world, so LIM-e2 (a WRENCH breach mid-path) is dropped. Open: how the rig turns the limit on (critic M2).
