# pin_cable_rates (4 Oct 2026, overnight): start the planner from the measured rod rates

Wesley, 3 Oct (overnight loop): remove the ring's height and xy errors from internal and external
model errors with a principled control-law change.

## 1. Question

If node 0 of the load planner pins each rod's angular rate r_i to its measured value (from the
mocap drone and ring velocities), as it already pins the ring pose and twist and the rod
directions, does the twin's +4.8 cm hold with z_ki 0 go away?

## 2. Hypothesis

**Measured (no flight).** `tools/hold_replay.py R0887` at a hold tick (ring 1.043 m, rods 49.4-50.4
deg, the drones at rest, |v| 0.004 m/s in run.csv): the converged plan's node 0 has rod rates of
0.29-0.33 rad/s. That means each drone already moves at 0.19 m/s, 0.14 m/s outward and 0.12 m/s down:

| slot | r (rad/s) | implied drone velocity (m/s) |
|---|---|---|
| 0 | (-0.006, 0.333, 0.005) | (0.141, 0.005, -0.120) |
| 1 | (-0.286, -0.151, -0.011) | (-0.061, 0.122, -0.119) |
| 2 | (0.281, -0.152, 0.008) | (-0.061, -0.119, -0.118) |

**The mechanism.**
- Node 0 pins the load pose and twist and the rod directions s_i. The rates r_i and tensions t_i are
  free (`observed_state_indices`, planner_ocp: pinning the stale resampled tension ratcheted it).
- So every plan starts the correction with a drone velocity the drones do not have. Its predicted
  motion assumes the move is already under way: rods rotating from 50 to 45 deg within 0.5 s, then
  the ring down.
- The trackers start from rest, and 0.1 s later the next plan re-pins the directions and assumes the
  jump again.
- The steady state is where the trackers' response to an always-restarted plan balances. That is
  the +4.8 cm (twin) and +6.3 cm (SIL, matched plant).

It explains the observer results:
- The one-step ring-velocity innovation reads zero, because the first move is in r, not v
  (R0880 +7.8, R0875 -12.5).
- A 0.5 s window reads zero too, because the ring moves only after 0.5 s (R0887 +4.4).

**The fix, standard MPC.** The prediction starts at the measured state (Rawlings, Mayne & Diehl
2017). r_i is a kinematic state, measurable from velocities:

    r_i = s_i x (v_attach_i - v_pivot_i) / l_i

with v_attach_i = v_L + R (w x rho_i).

The tension of a rigid rod is a constraint force that can jump, so t_i stays free (no ratchet: the
23 Sep ratchet came from pinning a stale resampled tension, not a measured state). rd_i and rdd_i
stay free (they are accelerations).

**Expected.**

| arm | without | expected |
|---|---|---|
| twin hold | +4.8 cm | within 1.5 cm, no integral |
| SIL hover (matched plant) | +6.3 cm | within 1.5 cm, no integral |
| ring tilt | | unchanged |
| solve failures | | none |

The push and wind still need a force estimate (option A); this fixes the internal error.

## 3. The one variable

Planner knob `pin_cable_rates` (bool, default false, passed only when set):
- node 0 also pins r_i, computed in `build_x_init` from the measured drone (pivot) velocities and the
  ring twist;
- a separate compiled OCP (different node-0 constraint set, its own model name and cache signature).

Everything else is pinned to each arm's config.

## 4. Baseline

R0841 (twin hold, z_ki 0), R0473 (SIL long hover), R0832 (orbit), R0856 (push).

## 5. Pass / fail numbers

| arm | supports | falsifies |
|---|---|---|
| SIL hover | within 1.5 cm of 0.600 | > 3 cm, or solve failures, or oscillation > 2 cm p-p |
| twin hold | within 1.5 cm of 1.000, tilt within 0.3 deg of R0841 | > 3 cm, or any solve failure, or an abort |
| orbit | xy lap mean no worse than R0832 + 0.5 cm, z within 1.5 cm | xy > R0832 + 1 cm |
| any | takeoff, lift and LAND complete | the tension ratchet (planned tension climbing over the hold), an abort |

## 6. Repeats

Exploratory, one run per arm.

## 7. Cost

1 SIL + 3 Gazebo runs (unlimited tonight).

## 8. New code?

Yes:
- `observed_state_indices(n, rates=False)`.
- `generate_load_ocp(..., pin_rates=False)`, with the model name and cache signature.
- `build_x_init(..., drone_slot_vel=None)`.
- The planner knob and its launch plumbing.

If it loses, it stays behind the off knob until Wesley decides.

## Critic

**Critic, 4 Oct 2026: NOT READY.** Rewrite the card. R0868 reversed the rod error and left the
height where it was, so the mechanism needs the free offline replay before any flight.

Must-fix:
1. Offline replay with the pin on R0841/R0868: node-0 r, rd, acceleration, model plant.
2. The sign of R0868's node-0 r against R0887's.
3. Arms: FF 0.9, n=2, circle, rod-model twin; the 1.5-3 cm band; p-p and the lift transient.
4. A missing velocity: wide bounds, not 0. On a reseed, the measured r at node 0 only.
5. Refuse in attach, dissipative, network and real modes. Prebuild `_pr` for n=2-4.
6. Log solve time with solve_budget 0.06.

**Must-fix 1-2 done (registry M-pin-replay).**

| run | rods | ring | drones in the free plan's node-0 jump |
|---|---|---|---|
| R0841 | 50 deg | +4.8 cm | down 0.12 m/s and outward |
| R0868 | 42.8 deg | +5.0 cm | up 0.04-0.06 m/s and inward |

The jump reverses sign at the same height error, so it is not what holds the ring high.

Pinned, both plans start from rest. The thrust feedforward then carries the move: node-0 thrust
accel 14.5-14.7 against 15.5 free. The model plant still settles at 1.000.

**Verdict: falsified offline, not flown.** The code stays behind `pin_cable_rates` (default off)
until Wesley decides.
