# Newcomer LAND and abort (M1 rules a and b)

Revision 2 (critic: REWRITE, 8 Oct night; Wesley's answers the same night). Rule c (a fault on QUAD4 alone) is its
own card: 2026-10-09_newcomer_fault_scope.md.

1. **Question.** Wesley, 8 Oct: "LAND should land all drones no matter which controller it is using"; an unwelded
   QUAD4 on LAND, or on an abort, moves clear and lands while the carriers fly on. Does the attach graph do that?
2. **Hypothesis.** Today (rig-prep 5130c70, read from the code): LAND leaves the newcomer's approach hold following
   the plate down, the landed gate counts only the carriers and `_departed_hold`, `/fleet/landed` fires and the
   fleet manager disarms QUAD4 ~0.6 m above the ring; `/magnet/command OFF` only blocks the weld and QUAD4 hovers
   over the plate until LAND. With `newcomer_lifecycle` on: LAND or OFF before the weld sends OFF, holds QUAD4 at
   its approach reference for 0.5 s (the magnet lets go), moves it radially out from the ring centre to 1.2 m at its
   height (0.15 m/s; Wesley: no climb, a climb can itself collide), lands it by itself, and the landed gate waits
   for it; OFF is re-sent every 0.5 s while it is out and any later ON or weld is refused with OFF.
3. **The one variable.** `newcomer_lifecycle` (bringup common.yaml, default false; planner param), branch
   newcomer-policy: dissipative_node.py `_abort_newcomer`, `_newcomers_out_on_land`, `_newcomer_off_tick`, OFF/ON
   handling in `_magnet_cmd_cb` (the subscription is no longer gated on the approach hold; ON keeps that gate), weld
   refusals in `_magnet_attached_cb`/`_do_attach`, `_publish_departed_refs` lands a `_land_alone` drone by itself
   (its own 0.15 m/s path also during LAND, its touchdown detector kept between ticks), `_pos` reads an unwelded
   newcomer's own mocap (stamped; older than 0.5 s is not used). Off in partner runs and outside the OCP phase.
   A newcomer whose approach never started gets no reference (R0532). Test tooling: runner `WAIT_LOG` (times an
   event into a planner phase), `tools/attach_metrics.py`.
   Pinned: configs/experiments/twin_r8a_dry.yaml (free-start twin attach world, M1 layout 270/30/150, QUAD4 on
   plate 9, hover 1.0 m, weld_radius 0) with `newcomer_lifecycle` the only difference between NC and A1.
4. **Baseline.** NC: the A1 config with the switch off, same tree. Expected: QUAD4 disarmed above 0.3 m at LANDED.
   If it lands cleanly the code reading is wrong and this card is void. R1238 (twin_r8a0, LAND before ATTACH):
   QUAD4 never had a reference, z 0.10 throughout, tilt 0; ring 0.999-1.001 m.
5. **Pass / fail.** Twin. Every row names the planner line that proves the event fell in the phase
   (`approach: drone 3 <phase>` before, `out of this flight (<why>)` after; `tools/attach_metrics.py`).
   Clearance = QUAD4 body to every carrier body / to every carrier rod (carrier -> plate), MAGNET ON -> LANDED.
   Tip-rim = QUAD4's magnet tip to the ring's plate circle while it moves out (the twin has no rim contact: the rod
   has no collision and the magnet does not collide with the ring, so this is measured, not felt).
   | Run | Events | Supports if | Falsified if |
   |---|---|---|---|
   | NC | switch off; MAGNET ON, WAIT_LOG descend, LAND +8 s | QUAD4 disarmed above 0.3 m at LANDED (the bug is there) | it lands cleanly: card void |
   | A1 | as NC, switch on | no `/fleet/abort`; out (LAND) logged in descend; moves out to >= 1.15 m with z within +/-3 cm of its hold; QUAD4 below 0.15 m and armed before LANDED; no 'not down after' line; clearance body >= 0.30, rod >= 0.15 m; tip-rim >= 0.02 m; ring tilt <= 10 deg; QUAD4 tilt <= 10 deg on the floor | QUAD4 disarmed above 0.15 m, any abort, body < 0.25 / rod < 0.10 m, tip-rim < 0.01 m |
   | A2 | LAND 2 s after transit starts | as A1 (out logged in transit) | as A1 |
   | A3 | LAND 2 s after the climb starts (at spawn, r 1.4) | as A1 (it is already clear: lands where it is) | as A1 |
   | B1 | weld_radius 0; OFF 8 s after descend starts; LAND 25 s later | out (abort) in descend; moves out and lands while the carriers hover (ring z +/-3 cm of 1.0, tilt <= 5 deg); LANDED after all four are down; A1's clearance and tip-rim bars | as A1, or the carriers move |
   | B2 | weld_radius 0.08; OFF 2 s after transit starts | no WELD event; as B1 | a weld |
   | B3 | weld_radius 0.08; OFF 1.5 s after descend starts, ON 1 s later | ON refused (logged), no WELD | a weld |
   | N1 | twin_r8a0 with the switch on (LAND before ATTACH) | QUAD4 never gets a reference, z <= 0.12, tilt <= 5 deg; ring as R1238 +/- 1 cm | a reference, or QUAD4 moves |
   Grey (between the columns): one repeat of that run.
   Not flown here (unit tests only): a weld racing LAND in the last 0.1 m of the descent (refused, OFF sent);
   the network-phase LAND (the switch does nothing outside the OCP phase); the drop path (drop is off in attach;
   its own card).
6. **Repeats.** A1 and B1: 2 each (Gazebo). NC, A2, A3, B2, B3, N1: 1 each. Neighbours before repeats (S8):
   A1 on an orbit, OFF after a weld (today's release, caught only by detection): 1 each once the rows above pass.
7. **Cost.** 10 Gazebo runs on the laptop (~7 min each), +2 neighbours.
8. **New code?** Yes, a gate that can hold the fleet down (the landed gate waits for an out newcomer; its own
   touchdown or the planner's existing 15 s fallback ends the wait). Behind `newcomer_lifecycle`, default off; it
   becomes the default (Wesley's standing rule) only after the rows above pass and the reviewer reads the table.
   Removal: drop the branch; R8a/R8b then fly with the sheet rule "no LAND while QUAD4 is airborne and unwelded:
   OFF, wait, then hand-fly it clear" instead.

## Critic

Verdict REWRITE (8 Oct night, on revision 1, which also held rule c). Must-fix and should-fix items, each with what
revision 2 does:
- M1 (c) not what Wesley decided; split the fleet-manager exemption into its own card. Asked: Wesley chose "QUAD4
  falls alone now, a tracker fault-land later". Split out: 2026-10-09_newcomer_fault_scope.md.
- M2 LAND must not give an idle floor newcomer a hold (R0532). Done: no approach, no reference; N1 bars QUAD4's
  reference, z and tilt.
- M3 the step-out near the rim: OFF, wait for the release, climb, step at <= 0.15 m/s; the twin cannot feel rim
  contact. Wesley: no climb (radially out at its height). Done: OFF, 0.5 s hold, level radial move at 0.15 m/s,
  also during LAND (not the fleet's 0.4 m/s clear-out); tip-rim measured from the tip columns.
- M4 refusals must reach the magnet: the runner repeats ON for 2 s and the magnet manager welds on its own. Done:
  OFF re-sent every 0.5 s while out; a refused ON or weld publishes OFF.
- M5 B1 as written could not exist (it welds during the descent). Done: B1 at weld_radius 0; every row timed with
  WAIT_LOG on the planner's phase line and checked against it after the run.
- M6 the FAULT injector was a service shutdown. Moved to the fault-scope card (push world + pose-loss event,
  Wesley's word given).
- M7 partner mode. Done: the switch is forced off in partner runs (LAND there goes to Tejen's land_now, combined
  stack card).
- M8 on_load semantics. Moved to the fault-scope card.
- S1 ATTACH meant MAGNET ON. Done. S2 R1238 row and metrics.json numbers. Done (registry R1238). S3 behind a
  parameter, default off, NC on the same tree. Done. S4 ARM before TAKEOFF clears 'out'. Done. S5 own OFF echo is a
  no-op (tested). S6 attach_moving false clears the approach hold. Done. S7 stamp attach_pos. Done (0.5 s).
  S8 neighbours: listed in 6, the weld-LAND race as a unit test. S9 clearance with the rods. Done (attach_metrics).
  S10 the 'not down after' line and QUAD4's floor tilt. Done (bars). S11 drop path and network LAND out of scope.
  Done (the drop edit removed; the switch acts only in the OCP phase). S12 'a later weld is refused': B3 and unit
  tests.
