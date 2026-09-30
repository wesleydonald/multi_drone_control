# Pretension before the lift (floor start), 2026-09-30

**Question** (Wesley's supervisor): can the drones reach a similar rod tension at the same time before the ring
leaves the floor, instead of pulling individually?

**What failed on the rig today** (registry RIG-0930-*):
- **R3b0 f1:** the tracker zeroed the cable feedforward while the ring rested (z <= 0.10), so it deadlocked.
- **f2:** only drone 0 subscribed to `/payload/desired_position`, so drone 0 alone dragged the ring off the floor (12.9 deg tilt, jump to 0.91 m).
- **f4:** a creep timeout at 25.6 deg with the lift started anyway; the drones pulled one at a time, and 3 magnets released.
- **lift f5:** the planner eased the FF 0->1 over 1.0 s from the ramp start, but the tracker gate (desired > ring + 3 cm) opened at +1.1 s. The full pull arrived as one step on all four: ring launched at 0.29 m/s, 21 deg tilt, 2 magnets released (rated 10-12 N).

**Change (the one variable: how the rod pull is applied on a floor start):**
1. **planner_node:** after the handover settle, a pretension phase.
   - `_ff_t` ramps 0 -> 1 over `pretension_s` (default 3.0; a param and launch arg) with the lift ramp held; then full pull for 0.5 s; then the height ramp.
   - Air starts (`start_taut`) are unchanged (1 s ease during the ramp).
2. **planner_node:** publishes latched `/fleet/cable_ff_active` (ff > 0).
3. **controller_mpc:** zeroes the cable FF on a resting ring only while that flag is false. The previous tracker-side gates are removed: the resting-only gate and the desired-height gate.
4. **planner_node:** after a creep 'elevation timeout' on a floor start, it refuses to pull or lift, logs an error and shows 'creep timed out: LAND'.

**Prediction / falsifier:**
- **Gazebo:** four throttles rise together during the pretension, and the ring breaks away level (tilt < 5 deg) with no jump. Falsified by a throttle spread > 0.02 at any tick of the ramp, ring tilt > 5 deg, or an abort.
- **Rig:** all four "thr" rise together over 3 s and the ring floats up level. Falsified by one side leaving first, or any magnet releasing.

**Baseline:** R0765 (Gazebo, the desired-height gate), RIG-0930-lift-f5.

**Result so far:** R0766 (Gazebo), PASS.
- Throttles 0.119 -> 0.157 together over 3 s, identical to 3 decimals.
- Ring +1.6 cm at the end of the pretension, tilt peak 0.53 deg, hold 0.600, landed.
- Rig: pending.

## Critic

Verdict: **not ready** (critic agent, 2026-09-30). The critic had read-only tools; its challenge is summarised here with the file:line it cited.

1. **Tried before?** The refusal as first written is the hold-f1 slide: only the clock stopped and ff was zeroed, but `_plan` still solved and published the OCP horizon at the nominal 45 deg, dragging the rods up the arc.
2. **What the change did not address: the per-drone tautness gate still scales each rod's pull** (`gate * ff * cable`; gate = clip((dist - 0.85 L)/(0.15 L))).
   - Rig resting distances are 0.42-0.50 m against a typed 0.47, and measure_rod_len rejects them.
   - So on the rig the rods ramp to different fractions of their share. At breakaway the gate jumps to 1 as a step on one side: "one side leaves first", which Gazebo (rigid rods, gate 0.9999) cannot show.
   - The kT/pack mismatch means that with over-thrust the ring leaves mid-pretension.
3. **Falsifier.**
   - A throttle spread cannot fail in Gazebo for the rig's reasons and fails on 1/3/5/9 by design. Use the ff·gate spread instead.
   - The rig falsifier needs numbers: tilt > 5 deg during the pretension, vz > 0.10 m/s before the ramp, any magnet release.
   - The card changes four things at once; fly the refusal as its own arm.
4. **What it breaks.**
   - (a) The refusal slide (item 1).
   - (b) The flag is never cleared on an OCP LAND (`_land_hold_refs` returns before `_publish_refs`). The tracker then passes the land blend's c_ff to a resting ring: the R0113 mechanism.
   - (c) The dissipative network never clears the flag, so on network LAND the full cable FF reaches grounded drones (regresses R0108/R0109).
   - (d) M2 (`start_taut` false, `airborne_start` true) would pretension and could refuse.
   - (e) `lift_ramp_vel` 0 is no longer a no-pull hold, and the phase reads "lifting" forever.
   - (f) `lift_z0` is not re-latched after the pretension, so the OCP pulls a floated ring back down.
   - (g) The refusal fires at any timeout, even 36.9 deg.
   - (h) Refused + LAND lets ff back in during the descent.
5. **Scope:** yes (the supervisor's question; the refusal proposed in RIG-0930-R3b0-f4).

**Must fix:**
1. The refusal holds the drones in place with no OCP and no cable term.
2. Equal pull (gate 1) during the pretension, and log the gates.
3. Clear the flag on LAND and on network entry.

**Should fix:**
- Re-latch `lift_z0`.
- Skip the pretension on a hold.
- Rewrite the falsifier.
- Fly the neighbours: 1/3/5/9, a forced timeout, M2.

### Response (same day)

- **Must 1:** `_refused_hold_refs` holds each drone at its position latched at the refusal: level, hover thrust, no cable, flag false; `_plan` returns before the OCP. ff = 0 whenever refused, descending included.
- **Must 2:** on a floor start after the settle, every rod gets gate 1 (rods rigid after a hand-over at 37 deg or more). The length gates are logged at the start of the pretension.
- **Must 3:** `_set_ff_active(False)` on entry to the OCP LAND hold (load down) and on `_enter_network_phase`, so the tracker's resting-ring gate does its old job there.
- **Should:**
  - `lift_z0` re-latched to the ring height at the end of the pretension if the ring rose.
  - `lift_ramp_vel` 0: no pretension and no pull; the phase reads "holding (no lift)".
  - Airborne starts (M2) never pretension or refuse.
  - The refusal only fires below 30 deg (`REFUSE_BELOW_DEG`).
- **Neighbours flown in Gazebo:**
  - even ring (regression);
  - 1/3/5/9 (`rig_lift_n4_3915_floor`);
  - forced timeout (`rig_lift_n4_even_floor_timeout`, typed rod 0.30 on 0.5 m sim rods).
  - M2 not re-flown: the airborne start is excluded from both changes, and its ring is never resting in flight.
- **Rig falsifier:** during the pretension, ring tilt > 5 deg, ring vz > 0.10 m/s before the height ramp, or any magnet release, is a FAIL.

### Gazebo after the critic fixes (headless, 2026-09-30)

| run | arm | ring tilt peak | pretension throttle spread | hold z | verdict |
|---|---|---|---|---|---|
| R0767 | even ring, rig settings | 0.37 deg | 0.000 | 0.600 | PASS |
| R0768 | plates 1/3/5/9 | 0.92 deg | gates 1,1,1,1 (shares differ by design) | 0.600 | PASS |
| R0770 | forced timeout at 33.8 deg (>= 30) | 0.00 | lifted by design, ring 0.198 (wrong typed rod) | 0.198 | refusal not exercised |
| R0771 | forced timeout at 28.2 deg | 0.00 | refused: drift <= 1 mm in 25 s, ring untouched, LAND clean | 0.010 | PASS |

Verdict: ready for the rig pending the rig falsifier:
- ring tilt > 5 deg during the pretension;
- ring vz > 0.10 m/s before the ramp;
- any magnet release.

M2 was not re-flown: airborne starts are excluded from the pretension and the refusal.
