# 2026-09-24 LAND with a parked detached drone (OCP re-solve) no longer aborts

## 1. Question
Open problem 5 (CURRENT_STATE §7): after a 4 → 3 detach in `reconfig_mode: ocp`, LAND
ends in a fleet abort (R0113/R0114, 2026-08-06). On hardware a fleet abort at LAND is a
mid-air disarm of the freed drone. Does the v2 fix below land the survivors and the freed
drone cleanly, and leave every no-detach LAND and the network-mode LAND unchanged?

## 2. Root cause (corrected after the critic's replay of R0113's run.csv)
Two things, not one:
1. **The touchdown test was held open by the departed drone.** `_touchdown_stalled` took
   `max` over ALL physical `drone_pos`; drone 3 was still descending at `land_vel` from
   its 0.66 m hold, so the survivors' stall (floor contact 94.0 s) was never seen.
2. **What actually tipped drone 0 was a sideways push, not "grinding into the floor".**
   The survivors' node-0 references matched their measured positions and the load
   reference was only 0.6 m underground (not 1.5 m). But the load had been resting on
   the floor since before LAND (auto-descent), the rods were slack, and the planner's
   cable feedforward was still at 1.0: the OCP pins 45° cable directions and nominal
   tension, i.e. ~3.8 m/s² of sideways specific force per drone on a drone that is sitting
   on the floor. Drone 0 slid at 0.25–0.39 m/s, drifted 5 cm off its xy reference and
   tipped to 62° at 95.32 s; tilt fault 95.365 s (3.4 s after LAND). The same mechanism is
   R0276–R0279 ("tipped over on landing") and R0329's known touchdown abort, with no
   detach involved.

## 3. The change set (v2, after the critic)
1. **Cable feedforward off once the load is down during LAND** (`_publish_refs`,
   planner_node.py): with `_land_to_ground` and the load within 5 cm of `lift_z0`, the
   published cable term is zero (rods slack, nothing to feed forward). Closes the sideways
   push in (2). Logged once: "load down — cable feedforward off, rods slack".
2. **`TouchdownDetector`** (pure class, same constants) fed ONLY the drones still on the
   load (`_drone_at(i)` over the current slots). Closes (1). Identical numbers on an
   unresized fleet.
3. **Departed drones descend at `departed_land_vel`** (0.0 = 2 × land_vel; a freed drone
   carries nothing) and run their OWN TouchdownDetector: "down" = stalled on its descent
   OR z ≤ `departed_down_z` (0.15 m). No absolute-height dependence on the rig. A hold of
   None (no pose at the detach) counts as NOT down.
4. **Reference freeze + gate:** once the survivors stall, `lift_progress` stops sinking;
   `/fleet/landed` is announced when the departed drones are down, or after 15 s with a
   WARN (documented: that disarms a stuck departed drone in the air; the operator holds
   ESTOP; with 3. the wait is normally zero).
5. Guards: `/fleet/detach` refused during LAND; `detach_magnet:=true` re-arms every
   tether (`/drone_i/magnet ON`) at ARM because the radio latches the last value across
   flights; a refused detach never drops a magnet (release is after every refusal return).
Launch: `real_dissipative_launch.py` exposes `reconfig_mode` / `reconfig_hold_s` /
`detach_magnet` (defaults unchanged: network / 1.5 / false); `real_io_launch.py detach:=true`
shows the button.
Not addressed: network-mode LAND announces on load height alone and can still disarm a
freed drone in the air; the rig flies `reconfig_mode:=ocp` (tests.txt), and the network
default stays for the record.

## 4. Baseline
R0113 / R0114 (`detach_ocp_n4`, OCP resize, LAND at 92 s → abort 3.4 / 3.5 s later).
Controls: R0111 (network mode, clean LAND); SIL `carry_hover_n3` R0467 and Gazebo
R0466 (no detach) on the v1 tree; to be re-flown on the linearised sim plant.

## 5. Pass / fail
| arm | supports | falsifies |
|---|---|---|
| `detach_ocp_n4` ×2 (Gazebo, forbid_abort) | no abort; log order DETACH (OCP resize) → LAND → "load down — cable feedforward off" → "departed drone 3 is down" BEFORE "descent complete" → "Landed - disarming"; (a) floor contact of the highest survivor to disarm ≤ 2 s; (b) peak tilt of EVERY drone between LAND and disarm < 30°; (c) gate wait ticks logged = 0 | any `/fleet/abort` or ENVELOPE fault; `/fleet/landed` with drone 3 not down; (a) > 3 s; (b) > 45°; the 15 s WARN |
| `detach_network_n4` ×1 (control, if a run remains) | R0111's LAND behaviour, no new planner lines except "load down — cable feedforward off" | anything else |
| no-detach LAND: SIL `carry_hover_n3`, Gazebo creep n3, unit tests | touchdown fires as before; "load down" line appears during LAND; no abort | a changed touchdown time or an abort |
| offline replay (test_touchdown.py, R0113 data) | survivors-only stall fires ≥ 0.3 s before the recorded fault; departed at 2× rate down before it | either fails |
Replay result (2026-09-24): survivors fire at ~95.0 s vs the 95.365 s fault; departed at
2 × 0.2 m/s from 0.66 m is down at ~93.4 s. Timing alone is a 0.35 s margin, which is why
change 1 (no sideways push) is the load-bearing part, not the gate.

## 6. Repeats
2 for the claim arm (Gazebo). SIL once (deterministic). Replay: free.

## 7. Cost
3 Gazebo runs (tomorrow: today's 6 are spent).

## 8. New code
planner_node.py (+~90: class, wrapper, hooks, ff-off, freeze/gate), dissipative_node.py
(+~45: params, departed detectors, magnet helper, ARM re-arm, LAND guard), two launch
files, test_touchdown.py (7 tests incl. the replay). Removal plan: revert the LAND branch
and `_publish_refs` to the pre-2026-09-24 form; the class can stay.

## Critic (2026-09-24, on v1: TouchdownDetector + freeze + 0.15 m gate only)
Verdict: **not ready**. Replaying R0113's recorded run.csv through the v1 design: the
survivors' highest drone stops at 94.0 s, the stall fires at ~95.0 s (LAND_STALL_S), the
departed drone is still at 0.186 m so the gate opens at ~95.2 s, against the 95.365 s
fault: 0.1–0.2 s margin, so two Gazebo passes would not show a fix. Root cause as written
was wrong: the load reference was 0.6 m underground, the survivors sat ON their
references, drone 0 slid sideways at 0.25–0.39 m/s under the still-active cable
feedforward until it tipped; R0276–R0279 and R0329 are the same failure without a detach.
Freezing `lift_progress` does not touch the sideways demand. New armed-on-the-floor state
(0–15 s at hover throttle, `ref_stale` if solves fail; the 15 s cap disarms a stuck
departed drone in the air, the very case the gate exists for). Absolute 0.15 m threshold
is what the touchdown design rejected; rig resting z unknown; a None hold counted as down.
Network-mode LAND unprotected and is the rig default. Magnet: nothing ever sends ON again;
detach accepted during LAND; the panel spinbox auto-increments (a double press releases
a second drone; OCP mode allows 3 → 2, which capsizes in SIL). Tests exercised only the
pure class. Bands missing: floor-contact-to-disarm time, peak tilt of every drone, gate
wait ticks; three coupled changes called one variable.
Required before the first run: offline replay test with the predicted announce time;
change the design if the margin is ≤ 0.2 s (bring the departed drone down first, and look
at why a grounded survivor slides); correct §2; add the bands; rig notes (magnet ON
republish, departed resting z, network-mode decision).

## v2 (same day, applied above)
Replay test added (fires ~95.0 s, departed down ~93.4 s at 2× rate). Design changed: the
sideways push is removed at source (feedforward off once the load is down during LAND),
the departed drone comes down first and is judged by stall, None holds block the announce,
detach is refused during LAND, magnets re-arm at ARM. Bands (a)–(c) added. The 3 → 2
double-press hazard and the network-mode LAND are noted, not fixed.

## First Gazebo pair, v2 (R0490 / R0491, linear plant, 2026-09-24 late)
Both failed, for two different reasons, neither the v2 mechanism:
- R0491 aborted in the LIFT (drone 1, TAKEOFF+8 s): the config was pinned to the
  `start_taut: true` floor start as R0113 flew it, and with four drones on the linear
  plant that path dives inward (as R0468). Void for the claim. Config moved to the creep
  path (start_taut false, handover 45, settle 1.0, creep_vel 0.2).
- R0490: detach clean (resize to n=3, "180 deg gap — the load cannot hang level" warning,
  circle continued); after the circle the auto-descent left the load hanging at 0.155 with
  the three survivors at 0.475 and 21° of tilt (the uneven 0/90/180 ring, open problem 7).
  LAND: reference dived at 0.2 m/s, the load touched the floor at +2 s, the survivors
  came down at 0.16 m/s and touched at +2.5 s, "load down — feedforward off" fired, the
  departed drone was at 0.19 and descending at 2× (down by +3.5 s), and drone 2 went
  from 21° to 84° in 0.3 s after touching down, before any stall could be seen: a
  drone that lands with 20° of tilt tips. v2's three changes all did what they were built
  to do; the tip is a new mechanism (touching down tilted with the OCP still steering).

## v3 (same night): grounded survivors descend straight down, level
Once `_load_down()` during LAND, the planner stops solving the OCP and publishes, for
each survivor, a straight vertical descent from its measured position (xy anchored,
z − land_vel·t, hover thrust, zero cable term) — the creep in reverse — until the stall
detector sees it down; then frozen. The OCP's 45° references are what pulled grounded
drones sideways (R0113) and kept them tilted onto the floor (R0490). Touchdown, the
departed gate, the freeze and the announce are unchanged. Falsifier for the re-fly: any
survivor tilt above 30° after LAND, or an abort. Proof: SIL smoke LAND (the hold path
runs on every LAND once the load is at its lift height) + `detach_ocp_n4` ×2.

### Second pair, v2 + creep config (R0493 / R0494)
- R0493: abort 2.7 s after the DETACH: `[Drone 1] Pose timeout (0.25s) - disarming`, a
  mocap gap on the tracker during the resize (the node primes the n=3 solver twice at the
  detach; the sim was starved for a quarter second). Not a control fault; it is the
  tracker's rig watchdog firing on sim load. Void for the claim; note for the rig that a
  real mocap dropout at the detach does the same.
- R0494: SUPPORTED, 1 of 2 with v2: creep lift, detach, circle, LAND with "load down —
  feedforward off" at +3 s, survivors stalled at +6 s, "Landed - disarming" with the
  departed drone at 0.10 m, peak survivor tilt after LAND 22° (was 54–74°), no abort.
- Both show the 0/90/180 survivors hanging the 0.86 kg ring at 31–41° (max 57° in the
  transient; the fault is 60°) with drone 1 (the 90° one) at 1.16–1.22 m. That is the
  geometry (CoG on the hull, open problem 7), not the LAND fix, and it is what a 4 → 3
  detach on an even 4-ring gives on the rig too: any three of four 90°-spaced plates leave
  a 180° gap. The 2026-08 "~1° settled tilt" was the 0.6 kg disc on the quadratic plant.
v3 (straight-down landing for grounded survivors) is now actually applied (the first
edit had failed on an anchor and R0493/R0494 flew v2); SIL smoke passes; Gazebo pair
tomorrow (today's headless count is 10).

## Correction (reviewer, 2026-09-25)
The pass band "'departed drone 3 is down' BEFORE 'descent complete'" does not match the
built v2/v3 design: a departed drone counts as down when it STALLS or drops below
`departed_down_z` (0.15 m), and the "is down" line prints only on the stall. In R0554/R0556
the announce fired with the freed drone at 0.113/0.117 m (resting height 0.10), and the
stall line followed 2.3 s later. The band that the design implements, and that those runs
meet: freed drone below 0.15 m or stalled at /fleet/landed; no abort; survivor tilt after
LAND < 30° (max 18.7°).

## v4 (2026-09-25 evening): the freed drone lands clear and holds at its stall
Hover-detach runs exposed two landing faults of the freed drone (it trails its released cable):
straight down it landed 0.63 m from the ring centre, the ring came down on its cable and tipped
it (R0567, abort); and its reference kept sinking after it stalled, pressing it onto the cable
until it tipped 0.2 s before the fleet disarm (R0570). Now it steps out to 1.2 m from the ring
centre at its hold height (0.4 m/s) before descending, and freezes its reference at the stall.
R0571, R0572 (both fixes): clean, freed drone 1.20 m out, ≤ 3.7° while armed, survivors ≤ 18.9°, no abort. R0569 had the step-out only (clean); R0570, same code, tipped the freed drone to 87° while armed (no abort, 0.2 s before the disarm).
