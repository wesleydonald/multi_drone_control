# Newcomer LAND, abort and fault (M1 rules a, b, c)

1. **Question.** Wesley, 8 Oct: "LAND should land all drones no matter which controller it is using"; "a fault can
   cause a land"; an unwelded QUAD4 on LAND steps 1.2 m clear and lands, and a fault on QUAD4 alone lands QUAD4 while
   the carriers fly on. Does the attach graph do that for an unwelded newcomer?
2. **Hypothesis.** Today it does none of it (read from the code at rig-prep c6ff2d1, no run yet):
   - (a) LAND: the newcomer's approach hold follows the plate down, the landed gate counts only the carriers and
     `_departed_hold`, so `/fleet/landed` fires with the newcomer ~0.6 m above the ring and the fleet manager disarms
     it in the air (falls onto the ring). A weld during LAND is not refused and resizes the OCP mid-descent
     (`_land_hold_refs` then indexes 4 anchors built for 3: KeyError, every tracker ref-stale, fleet stop).
   - (b) Abort: nothing aborts an approach. `/magnet/command OFF` only blocks the weld; the newcomer hovers over the
     plate until LAND.
   - (c) Fault: tracker_3's envelope or pose timeout disarms it, the fleet manager sees an in-flight disarm with mux
     state 'ours' and stops the whole fleet; the carriers drop the ring from the air.
   With the change: (a) the newcomer steps level 1.2 m from the ring centre along its plate radial and lands, and the
   landed gate waits for it; (b) OFF before the weld does the same (magnet OFF first) while the carriers keep flying;
   (c) a disarm of an unwelded newcomer is not escalated, the carriers fly on, a later weld of that drone is refused.
3. **The one variable.** The newcomer's lifecycle code, nothing else:
   - dissipative_node.py: `_pos(d)` (drone_pos, else attach_pos) in `_departed_down`, the departed touchdown detector
     and `_drop_and_land`; `_abort_newcomer(j, why)` (OFF first, approach popped, departed hold from the approach's
     own reference `p_ref` so the hand-over is bumpless, step to 1.2 m along the plate radial at the hold height, a
     per-drone `_land_alone` so it lands with `departed_land` off); `_magnet_cmd_cb` handles OFF and refuses ON
     during LAND / after an abort (subscription no longer gated on `attach_traj_hold_s`); LAND calls the abort for
     every pending newcomer before the fleet LAND; `_do_attach` / `_magnet_attached_cb` refuse an out newcomer and
     any weld during LAND; `/drone_{d}/arming_state_feedback` edge on a reserved, unwelded drone marks it out
     ('fault'), sends OFF and drops it from the landed gate; latched `/drone_{d}/on_load` (False until its weld).
   - fleet_manager_node.py: in-flight disarm of a drone whose latched `on_load` is False: warn, do not escalate.
     Carriers never publish `on_load`, so carry, detach and partner runs are unchanged. Operator DISARM, ESTOP and
     the pre-TAKEOFF gate are unchanged (DISARM still disarms everything).
   - Runner (test only): a `FAULT <d>` event that calls `/drone_<d>/arming_service` with disarm, which is what a
     tracker safety disarm does (no world change).
   Everything else pinned to configs/experiments/twin_r8a0.yaml (free-start twin attach world, M1 layout 270/30/150,
   newcomer on plate 9, hover at 1.0 m).
4. **Baseline.**
   - R1238 (twin_r8a0, LAND before ATTACH): ring z 0.999-1.001 m at hover, tilt peak 3.5 deg, QUAD4 armed idle at
     z 0.10, clean LANDED. Must be unchanged (neighbour N1).
   - Negative control on c6ff2d1 (one run of A1 below before the change): expected newcomer disarmed above 0.3 m at
     `/fleet/landed`. If it lands cleanly the code reading is wrong and this card is void.
5. **Pass / fail numbers.** Twin, `weld_radius` 0 unless stated (no weld possible). Separation = newcomer body to
   the nearest carrier body, from run.csv.
   | Run | Event | Supports if | Falsified if |
   |---|---|---|---|
   | A1 | ATTACH, LAND in the descend hold | no `/fleet/abort`; level step reaches >= 1.15 m from the ring centre with newcomer z within +/-3 cm of its hold; newcomer z < 0.15 m and stalled before LANDED; separation >= 0.35 m throughout; ring tilt <= 10 deg | newcomer disarmed above 0.15 m, any abort, or separation < 0.25 m |
   | A2 | ATTACH, LAND during transit | as A1 | as A1 |
   | B1 | `weld_radius` 0.08, ATTACH, OFF in the descend hold, LAND 20 s later | no weld; step and landing as A1 while the carriers hover (ring z within +/-3 cm, tilt <= 5 deg); LANDED after all four are down | a weld, any abort, separation < 0.25 m |
   | C1 | ATTACH, `FAULT 3` mid-transit, LAND 15 s later | no `/fleet/abort`; carriers keep the ring within +/-5 cm and <= 10 deg; LAND lands the carriers; no weld | any fleet abort or carrier disarm before LAND |
   | C4 | weld (`weld_radius` 0.08), then `FAULT 3` | fleet-wide stop as today (`forbid_abort` false) | carriers fly on |
   | C5 | ATTACH, operator DISARM mid-approach | all four disarmed, muxes latched | any drone still armed |
   | N1 | twin_r8a0 unchanged | R1238's numbers within +/-1 cm, +/-1 deg | otherwise |
   Grey (between the columns): one repeat of that run.
6. **Repeats.** A1, B1, C1: 2 each (Gazebo). A2, C4, C5, N1, negative control: 1 each.
7. **Cost.** 11 Gazebo runs (laptop, ~7 min each, ~80 min). Unit tests per the change list first.
8. **New code?** Yes: a gate that can hold the fleet down (the landed gate now waits for the newcomer) and a change to
   fault escalation (amends the locked fleet-wide disarm for an unwelded newcomer only; Wesley decided it on 8 Oct,
   decisions line "M1 authority and safety"). Removal plan: revert the commit; the R8a/R8b rig rungs then stay
   blocked (do not fly R8a without rule a).
   **Open for Wesley:** rule (c) as built is "a fault disarms QUAD4 alone; the carriers fly on". The tracker cuts its
   own motors on every fault (tilt > 60 deg, speed > 3 m/s, pose lost > 0.25 s, reference stale > 1 s), and after the
   first three no landing is possible. A real "fault lands QUAD4" would need a tracker change (for example a free
   drone that descends on its last reference when the reference goes stale): not in this card.

## Critic

(pending)
