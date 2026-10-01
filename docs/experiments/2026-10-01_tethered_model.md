# Tethered model: solve failures, rod pivot, tracker throttle bookkeeping (card A), 2026-10-01

Plan: `docs/plan_2026-10_tethered_model.md` (Schedule and critic must-fixes override its draft).
Inputs: `docs/rig_2026-09-30_review.md`, `docs/rig_2026-09-30_replay.md` (W2),
`docs/rig_2026-09-30_force_balance.md` (W3). Code: branch `real-world-testing`, base 155ee56 plus
the uncommitted W1/W8/W9/W12/W13 edits.

**Question.** On the rig, can the planner fly a four-drone tethered hover with no solve
failures and no stale references, measuring its rods from the real pivot, and with the tracker's
pinned throttle state equal to the thrust actually produced? And does the ~80 % airborne force
balance depend on height or on the rod hanging in the downwash?

## Hypotheses (plan section 2, updated with W2/W3)

| # | hypothesis | status after W2/W3 (offline) |
|---|---|---|
| H1 | Heave and tilt spikes come from stale, stepped references (blocked single-threaded planner, held refs) | open; f7 heaved with 0 failures and the rebuilt refs oppose the heave (W2 `--heave`), so H1 cannot be the only cause |
| H2 | Solve failures come from an incomplete reseed (x only; u, slacks, multipliers survive) | **refuted as the trigger** by replay arm a (9/5 failures, unchanged). Inferred replay trigger = ring quaternion sign flip at yaw 180 deg (mocap sends w >= 0) together with the 30 deg slot offset of model-f1 (the ring quaternion was not logged on the rig, so the trigger is inferred). The open-loop replay of the 155ee56 planner gives 14 failures (9 + 5; 4 within 0.15 s of a rig failure) and 0 with the sign kept continuous (arm g); the shipped A4 (`planner_solver.py`) was replayed on the current tree with 0 in the base arm |
| H3 | Plant-side force deficit (ring lighter, ground effect, or the law under-reads when tethered) | open; masses trusted as measured (decisions 2026-09-30). W3: carried fraction rises with height across flights (f11 0.99 at 0.52 m vs 0.78-0.87 below 0.45 m); within flights the bands are not monotonic (f6 0.845/0.606/1.085/0.813/0.819; f7 0.723/0.879/0.807) |
| H4 | The planner over-plans (t_nom at 45 deg against rods at 54-66 deg) | weak: static hold asks 1.000 / 1.053 / 1.078 x W at 45 / 53.6 / 65 deg (W2), not 14-25 % |
| H5 | The 4 cm pivot is missing from the planner geometry (rods read 3-4 cm long, length gates 0.75-0.82) | code in (W9); replay arm c leaves failures at 9/5, so it is a geometry and gate fix, not a failure fix |
| H6 | FF inconsistency while ff < 1 (acc carries the full pull, cable scaled by gate*ff) | open, low rank (W10, not in this card) |
| H7 | Rod or pendulum mode (1.15 Hz tilt line in f10/f11) | report only |

## Arms (one variable each)

Each arm changes one thing against the base 155ee56 code. The other two arms stay at their base
setting when an arm is flown alone in sim.

| arm | the one variable | on / off switch | baseline |
|---|---|---|---|
| A1 (W8) | what the planner does on a failed solve: full reset on reseed (`solver.reset()`, x every node, u = 0), a failed SQP step ends the solve, wall budget `solve_budget_s` on every solve (rig 0.06, sim 0 = off; cold never cut below 5 iterations), a reseeded or cut solve with dynamics residual > 1e-2 held back and continued, last good horizon shifted by the time since it was solved for at most `max_shift_publishes` (3, allowed 0-4) node periods, then nothing | code (no switch); `max_shift_publishes:=0` restores "hold, publish nothing" | replay base (9 / 5 failures, 27 ticks > 100 ms, max 583 ms); RIG-0930-model-f1 (25 failures); SIL: last `carry_hover_n3` row at 155ee56 |
| A2 (W9) | planner rod geometry measured from the pivot `centre + R_i b`, b = [0, 0, -0.04] body; published refs moved back to the centre with the thrust attitude | `pivot_offset_z` launch arg (rig -0.04, sim 0.0) | replay arm c; RIG-0930-model-f1 (rods 0.587-0.613, gates 0.75-0.82); Gazebo R0767 |
| A3 (W12) | tracker `_applied_u[2]` / `last_cmd_throttle` = max(0, out - off(V)) after the real cap; model cap = throttle_max - off(V), refreshed <= 1 Hz; velocity-loop cap and kt_trim window on the model cap | code, active only when `thrust_offset > 0` (sim unchanged) | RIG-0930-model-hover (free hover within 1 cm), RIG-0930-model-f1 |

W13 (rig `z_i_max` 0.15, `z_i_gate` 0.25) is Wesley's word (decisions 2026-09-30), not an arm.
W10 (pull model) and W11 (freeze removal) are later arms of this card, not built yet.

## Predictions and falsifiers (stated before running)

| arm | test | prediction | falsified by |
|---|---|---|---|
| A1 | replay model-f1 (done, W2) | failures unchanged (trigger is the quaternion), ticks > 100 ms halve | already met: 9 / 5 failures, 27 -> 14 ticks. A1 is credited with latency only |
| A1 | SIL x1, `carry_hover_n3` air start | identical to baseline when no solve fails; 0 status != 0 | any status != 0, or payload-z trace differs from the baseline by > 1 mm with 0 failures |
| A2 | pytest (done) | rods = L, gate = 1, refs = centre | (passes, 26 tests incl. zero-offset identity) |
| A2 | SIL x1 and Gazebo floor x1 | on today's centre-pivot worlds `pivot_offset_z` 0 is a no-op; the only informative sim run is the mismatch neighbour: `pivot_offset_z:=-0.04` on a centre-pivot world | neighbour: rods read L - 0.04, gates < 0.9, creep hand-over within 45 +- 5 deg, no abort. Falsified by a creep stall, an abort, or tilt > 5 deg |
| A3 | pytest (done) | applied = out - off, cap follows V | (passes) |
| A3 | rig D1 (first closed-loop test; sim has no affine plant until W4) | takeoff to hover in the usual time + <= 0.5 s, model throttle within 0.01 of the law | takeoff > 1 s slower than RIG-0930-model-hover, a throttle overshoot > 0.05 over hover, or hover off the law by > 0.01 |

## Rig arms (next visit; every lift with W1 logging on; freeze on)

| # | test | launch args that differ from the rig defaults | pass / what we learn |
|---|---|---|---|
| D1 | one drone, free hover, rod and magnet removed (0.475 kg), 20 s | `thrust_ratio:=40.7` only (9.81 / (0.507 x 0.475); 35.2 is for 0.55 kg; real_hover_launch has no `drone_mass`) | model throttle vs the law at 0.475 kg; tells whether the rod in the downwash biased the fit. Doubles as the A3 takeoff check |
| D2 | tethered hold at ring 0.3 m, then 0.7 m, 15 s steady each (\|vz\| < 0.03 m/s for >= 5 s) | `z_ki:=0.0` | rod-force sum / ring weight at each height (`thrust_fit.py --tethered`); height effect vs a fixed law error |
| H1 | tethered hover 0.5 m, A1 + A2 + A3, 25 s | `z_ki:=0.0` | 0-1 solve failures; ref age > 0.3 s under 5 % of airborne time; heave p-p <= 6 cm; tilt mean <= 3, max <= 8 deg; hand-over 50-60 deg on the pivot reading (recorded, not a stop rule; was 45 +- 5, twin 55.5-55.7 with the creep fix, R0803-R0808); rods at hand-over 0.53-0.58 |
| H2 | as H1 with the safety net | rig defaults (z_ki 0.4, bound 0.15, gate 0.25) | H1 bars, \|mean z - 0.5\| <= 2 cm, \|z_bias\| <= 0.10 throughout |
| C1 | slow circle r 0.5 m at 0.125 m/s, net on; only if H1 and H2 pass | `load_traj:=orbit` | ring tilt mean <= 3 deg over the lap, \|z err\| <= 5 cm, 0 solve failures |

Pre-flight: packs >= 23.6 V and telemetry non-zero on all four; planner prints `pivot_offset=[0.0, 0.0, -0.04]`;
each drone within 10 deg of its modelled slot (model-f1 sat -28 to -31 deg off).
Stop rules (soft first: ramp the pull down and hold, then LAND): magnet release; ring tilt > 5 deg in the
pretension or > 12 deg in flight; ring vz > 0.25 m/s; 3 or more solve failures; creep timeout or refusal.
After one release read the log before the next lift; after two, no more lifts that session.

## Budget

Replay: done. SIL: A1 x1, A2 neighbour x1. Gazebo (laptop or lab): A2 neighbour floor x1. A3: none in sim
until the W4 affine plant exists. Rig: D1, D2, H1, H2, C1 (about 90 min).

## Critic

Challenge of the ACTUAL CODE (git diff 155ee56, working tree 2026-09-30 evening), written by the
critic for card A. The 41 new and edited tests pass (`test_solve_fallback.py`, `test_pivot_offset.py`,
`test_planner_log.py`, `test_thrust_offset.py`, `test_yaw_hold.py`, 0.7 s).

1. **Tried before?** The pivot lever is the closed 09-25 flip line (`decisions.md:33-35`, R0548 vs
   R0552): a 0.05 m arm pivot flipped the OCP-attached newcomer in Gazebo. W9 models the pivot's
   *geometry* only, not its torque; the torque is what flipped R0548. The reseed fix (A1) was tried
   offline: replay arm a leaves 9 / 5 failures (W2).
2. **Simplest explanation the change does not address.** The model-f1 failures come from the ring
   quaternion changing sign when its yaw crosses 180 deg (rig mocap sends w >= 0) while the drones
   sat 30 deg off their slots. `build_x_init` copies the raw quaternion (`planner_solver.py:173`)
   and node 0 is pinned to it, so the pinned state jumps to -q against a warm horizon at +q. Neither
   A1 nor A2 touches this. The one-line arm g fix (keep q in the hemisphere of the previous one) is
   not in the code, so H1 on the rig is expected to fail the solve-failure bar if the ring yaw sits
   near 180 deg again.
3. **Does the falsifier test it?** A1's failure-count claim is already falsified offline; the card
   now credits A1 with latency only. A2's sim runs cannot test the positive case: every carry world
   pivots at the centre until W5, so `pivot_offset_z` 0 runs are no-ops and only the mismatch
   neighbour is informative. A3 has no closed-loop test before the rig (sim `thrust_offset` 0 makes
   every W12 branch dead code); D1 is its first flight.
4. **What it breaks.**
   - *The reference watchdog (a gate that grounds the fleet).* The fallback is bounded only by
     `max_shift_publishes`, which has no upper limit: `shift_horizon` clips the shift at N
     (`planner_solver.py:307`), so any value >= 10 at 10 Hz keeps publishing a frozen last node past
     the 1.0 s watchdog, forever for large values. The streak counts failures, not elapsed time,
     so a shifted node 0 is misaligned whenever a failing tick overruns 0.1 s (replay: 100-550 ms).
   - *Solve latency.* The wall budget applies to reseeds only (`planner_solver.py:244`); warm solves
     that fail still run 5 RTI steps with up to 200 QP iterations each, which is where the replay's
     500+ ms ticks are (current tree: max 529 ms). The loop also keeps iterating after an RTI step
     returns status != 0.
   - *Unconverged "good" solves.* SQP_RTI status 0 only says the QP solved. After a reset, a budget
     cut at 1-3 steps is published as a good solve, resets the streak and zeroes `ref_age`. On a
     loaded machine (rig laptop with four trackers and RViz, lab slots) the cut comes earlier.
   - *SIL determinism.* The budget uses `time.perf_counter()`, so the cold iteration count, and so
     the trajectory, depends on host load in SIL and Gazebo. That breaks the "SIL is deterministic,
     one run per arm" rule and the W0 laptop/lab parity. The "sim unchanged" claim holds for W9 and
     W12, not for W8: every sim run's first cold solve is now cut to about 10 of 15 iterations.
   - *Tracker bookkeeping (A3).* `_applied_u[2]` can exceed the OCP throttle bound for up to 1 s
     when the pack voltage recovers (offset falls, realised = cap_now > cap_then), because
     `_refresh_model_cap` is rate-limited to 1 Hz (`controller_mpc.py:838`). The size is a few
     thousandths, but the node-0 state then sits outside the stage-1 bound. No test drives
     `_publish_channels` with a non-zero offset: `test_yaw_hold.py` stubs `_realised_model_throttle`
     to identity, so the spool path (realised 0 for the early spool, the builder's estimate is
     about 0.5 s slower takeoff) is untested.
   - *Pivot frames.* The sign and frames are right at every planner call site I traced: measured
     pivot `p + R_i b` with the mocap attitude (`planner_node.py:703-711`), published centre
     `pivot - R_ref b` with R_ref from the node's thrust vector (`thrust_vec / m`, which includes the
     cable pull), creep `pivot - Rz(yaw) b`. For b = [0, 0, -0.04] the yaw never matters. LAND hold,
     the refusal hold and the touchdown test use the measured centre in and publish the centre out,
     which is consistent. The zero-offset path is exact (`centre_from_pivot` and `_emit` return the
     input unchanged). But the rig attach and dissipative launches do not pass `pivot_offset_z`
     (default 0), so a rig attach flight silently runs without the pivot. The dissipative node's
     attach drones never store an attitude (`dissipative_node.py:418-423`), so their pivot assumes a
     level drone. Its network phase flies centres while its taut gates now read pivots.
   - *Tests.* `test_solve_fallback.py` covers `HorizonFallback` only. No node-level test shows
     `_plan_tick` publishes nothing after N failures, or that the arrival watchdog then trips. No
     test checks `max_shift_publishes` against N and the watchdog.
5. **Asked for?** A1, A2, A3 and W13 are in the plan Wesley approved. The quaternion fix (arm g) is
   not; the W2 report asks him.

**Verdict: rewrite the card (done above) and fix the must-fix items before the rig. A2 and A3 are
safe to fly after items 4-6. A1 must not fly until items 1-3 are fixed, because it touches the
reference-age gate.**

Must-fix:
1. `planner_solver.py:332-337`, `params.py:120`: bound the fallback by elapsed time, not by count.
   Publish only while `now - t_good <= max_shift * dt`, shift by `round((now - t_good) / dt)` nodes,
   and reject `max_shift_publishes` < 0 or >= N (or cap it so the last shifted publish plus
   `safety_ref_timeout_s` stays under 1.5 s). Add a node-level test: after the cap, `_plan_tick`
   publishes nothing, and the tracker's `_last_ref_wall` age passes 1.0 s by 1.3 s after the last
   good solve.
2. `planner_solver.py:237-245`: stop iterating on the first status != 0, and apply the wall budget
   to warm solves too (or cap `qp_solver_iter_max` for the warm path), so a failing tick cannot
   block the loop for 500 ms. Set a floor of STEADY_ITERS (5) cold iterations before the budget
   may cut, and log the KKT residual (`get_stats('residuals')`) in the tick log. A reseed that
   ends above a residual bound is a failure (fallback), not a good solve.
3. `planner_solver.py:244`, `params.py:117`: make the wall budget rig-only. Default
   `cold_budget_s` 0 (= off) in the sim launches, the SIL bench and `run_experiment.py`, and
   0.06 in `real_control_launch.py`. SIL and Gazebo then keep a load-independent iteration count
   (determinism and W0 parity).
4. `real_attach_launch.py`, `real_dissipative_launch.py` (next to `z_taut_gate`, lines 76 / 87):
   either pass `pivot_offset_z` with the rig default -0.04, or pin 0.0 explicitly with a comment
   that the attach/network path is legacy geometry. Wesley's call; today they differ silently from
   `real_control_launch.py:102`.
5. `controller_mpc.py:909-917`: clamp the realised model throttle stored in `_applied_u[2]` to
   `_model_thr_max`, or refresh the cap every tick (set_throttle_max is N `constraints_set` calls).
   Add one test that drives `_publish_channels` with offset 0.185 through the spool, and checks
   `_applied_u[2]` and `last_cmd_throttle`.
6. Card and rig sheet: D1 must launch with `thrust_ratio:=40.7` only (real_hover_launch has no
   `drone_mass`; 35.2 is the 0.55 kg gain); H1 and D2 with `z_ki:=0.0`. A3's pass bar is the D1 takeoff time against
   RIG-0930-model-hover.

Should-fix:
1. `planner_solver.py:173`: arm g (quaternion hemisphere continuity against `last_X` or the
   previous measurement), as its own arm A4 with the replay as its test. Without it the rig bar
   "0-1 solve failures" rests on the ring yaw staying away from 180 deg.
2. `planner_node.py:1010`: seed the post-failure reseed from the shifted last good horizon instead
   of a constant `x_init` on every node. That is fewer cold iterations and no reference step when
   the solve comes back.
3. `planner_node.py` tick log: add a column for what was published (solved / shifted / none).
   `solve_status` on creep ticks is the `_prime_solver` result, so a failure metric must filter
   by phase.
4. Pre-flight check in the planner: warn when a drone sits > 10 deg off its modelled slot (W2
   decision 1).
5. `dissipative_node.py:415-423`: store the attach drone's attitude like `_drone_cb` does, before
   any rig attach with the pivot on.

CRITIC: rewrite the card, which is done above, and fix the must-fix items. A1's shifted-horizon
fallback has no upper bound tied to the reference watchdog, and its wall-clock budget makes SIL
nondeterministic.

### Response

All six must-fixes accepted and applied (tests in `test_solve_fallback.py`, `test_planner_log.py`,
`test_thrust_offset.py`). Where the fix differs from the wording above:
- 1: `max_shift_publishes` allowed 0-4, not < N: the bound carries 0.25 node of timer slack, so the
  last shifted publish is <= 0.425 s after the good solve and the 1.0 s watchdog trips < 1.5 s.
- 2: the residual bound is on res_eq (dynamics), 1e-2: res_stat did not separate converged from
  cut solves in the model-f1 replay, res_eq did (cold reseed ~1 after 5 iterations, <= 1e-2 within
  0.5 cm of the converged refs, ~1e-5 after 15; warm <= 1e-5). A held-back solve is continued from,
  not reset, and the check stays on until one passes, else a budget-cut reseed would restart every tick.
- 4: attach and network rig launches pinned to `pivot_offset` 0 (legacy geometry) until should-fix 5.
- Should-fix 3 (published column) taken with the log change; should-fixes 1, 2, 4, 5 not done.

## Critic (phase 2)

Challenge of the phase-2 code (git diff 155ee56, working tree 2026-09-30 night): A4 (quaternion
continuity, slot-offset warning, dissipative attach attitude), W4 (rig thrust plant and pack), W5
(rig-twin worlds and models), W6 (tethered metrics, rig-twin configs, lab batch runner). The new
tests pass (`test_quat_continuity`, `test_rig_thrust`, `test_sil_rig_thrust`, `test_rig_worlds`,
`test_lab_batch`: 55 passed, 2.4 s). Nothing below was flown.

1. **A4 quaternion continuity: correct.**
   - `build_x_init` (`planner_solver.py:180-184`) keeps q in the hemisphere of the warm start's node 1,
     else of the last fed q. Node 0 (pinned to x_init) and the warm horizon now agree in sign.
   - Every other consumer gives the same result for either sign, so no new discontinuity:
     `quat_to_rot_np`, `yaw_from_quat` (atan2), the network's `_yaw_rot`/`_frame_rot`, the creep and
     the slot check.
   - The OCP attitude cost `e_att = 2 vec(q_ref ⊗ q*)` (`planner_ocp.py:85-86`) changes sign with q,
     but its square and its Gauss-Newton Hessian do not. So a state in the w < 0 hemisphere against
     `q_ref = yaw_quat(psi0)` costs the same, and the q_ref parameter does not need matching.
   - A reseed with a stale `_q_prev` (after LAND, or after a fleet resize at `planner_node.py:1218`)
     only picks a sign, and a reseed accepts either sign.
   - The dissipative subclass inherits `build_x_init` unchanged. The network is fed the raw mocap q
     and is sign-invariant.
   - The attach callback stores `[w, x, y, z]` from the same `MotionCaptureState.pose.orientation` as
     `_drone_cb`, in the same order (`dissipative_node.py:425-426`).
   - One gap. The tick log's ring-quaternion columns are the raw `load_state`, not the q the OCP
     was pinned to (`planner_node.py:562`). A replay reconstructs the pinned q correctly, because it
     re-applies the rule. A reader of the CSV sees the flip and not the fix. Report only.
2. **W4 thrust at the Gazebo motors.**
   - The law and its inverse are exact. `rotor_speed(T) = sqrt(T / (4 mc))` with mc 0.62e-6, the
     same value as `x3_rig_drone*.sdf:274`.
   - The rotor-speed clip (53.2 N) sits far above the rig law's maximum (about 16 N at u = 1).
   - The sign of the voltage term agrees with the tracker's `_thr_off` (`controller_mpc.py:818-822`).
     The plant uses 0.0219 and the tracker 0.022, which is negligible.
   - The thrust is exact only with zero rate-loop output. The mixer (`payload_betaflight_comm.py:246-255`)
     adds mc Σδ², and the armed floor 0.05·4631 gives 0.13 N below the offset, where the rig gives
     0 N. Both terms are the same as on the linear map, so they are acceptable, but the tests do not
     prove the thrust in Gazebo.
   - The first rig-map flight should check that a free-hovering x3_rig_drone at 0.475 kg sits at
     u = a_i + 0.507·0.475 − 0.0219·(V − 23.5), within 0.01.
3. **W4 pack voltage to the bridge and to the tracker.**
   - The two readings are consistent. Both come from the same `PackModel` in `sim_telemetry`: the
     bridge reads the true V (`/drone_i/sim/pack_v`, 50 Hz, sim time from the launch-wide
     `use_sim_time`), and the tracker reads the telemetry, quantised to 0.1 V at 10 Hz, as on the rig.
     SIL does the same (`bench_node.py`).
   - The hazard is the pairing. With `sim_thrust_map:=rig` on the control launch but not on the RViz
     launch, the bridge flies at a constant `pack_v0` and the tracker reads the placeholder
     voltage. The tracker's validity window (18-27 V) may silently take or drop the voltage term.
   - `_got_pack_v` is set but never read (`payload_betaflight_comm.py:115,145`).
4. **The rig-twin configs do not configure the tracker. Found by running the check.**
   - All six rig_twin configs pass `thrust_offset`, `thrust_offset_v_slope` and `throttle_max` to
     `mpc_quad_load_launch.py`. That launch declares none of them and does not pass them to the
     controller node (`mpc_quad_load_launch.py:306-325`), so `ros2 launch` drops them.
   - `run_experiment.py --check` still says `ok`. `declared_launch_args` (`run_experiment.py:139-142`)
     adds the arguments of every launch file whose name appears anywhere in the text, comments
     included. So mpc → rviz (a RUN ORDER comment) → real_io → `real_control_launch.py`, which
     declares all three.
   - Effect: the twin tracker flies the linear model, with kT 35.2, offset 0 and cap 1.0, against
     the rig plant. It commands u = a/35.2 ≈ 0.28 at hover where the plant needs about 0.46. With
     `kt_trim: false` the drones cannot lift, and every rig_twin run ends in LIFT_TIMEOUT.
   - W12 is dead code in the twin for the same reason, so the model_f1 description ("W12 ... in this
     tree") is wrong.
5. **The configs against the rig launch defaults.** I compared them with `real_control_launch.py`.
   Matching: cable_len, attach_radius, masses, handover_elev, creep, pretension, rod_tol/spread,
   z_i_max/gate, z_taut_gate 0.9, pivot −0.04, thrust_ratio 35.2, takeoff 0.0, kt_trim off, the
   offsets and the cap (once item 4 is fixed). The deliberate differences are target_z 0.5, z_ki 0
   and traj_speed 0.125 (the card). Not matched:
   - `payload_rest_z`. The rig uses 0.05 and the sim launch −0.1, so `payload_resting`
     (`controller_mpc.py:666`) is never true in the twin. On the rig it is true up to 0.10.
     The twin ring rests at 0.020, so 0.02 keeps the rig's +0.05 margin.
   - `attach_z`. The rig uses 0.0 and the twin 0.02. The twin value is right for its own
     mid-plane origin, but model_f1 is billed as "the planner as flown", and it is not on this
     parameter.
   - `thrust_v_ref` (23.5) and `solve_budget_s`. The sim leaves the budget at 0 by the critic's
     decision; state that in the config.
6. **W5 geometry: consistent.** I recomputed these from `four_rigid_ground_rig.sdf`:
   - The drone is at r 0.7746 and z 0.100, and the pivot at z 0.060. The joint pose `0 0 -0.04`
     is in the child (drone) frame.
   - Stub or magnet at (0.225, 0.040) on the top face. The pivot-to-magnet distance is
     √(0.5496² + 0.02²) = 0.5500.
   - The rod link axis is (sin 1.5344, 0, cos 1.5344), which is the magnet → pivot direction. The
     rod's top end lands on the pivot.
   - The COM (−0.1283) and Ixx 2.420e-3 match a 0.040 kg uniform rod plus 0.035 kg at the magnet
     end. The ring inertia matches the annulus formula (Izz 4.431e-2, Ixx 2.227e-2).
   - There are ball joints at both ends: payload → stub, and rod → drone. The rod and the magnet
     are visual-only, so nothing interpenetrates. The ring and drone collisions rest exactly on the
     floor: ring 0.020 ± 0.020, drone box 0.2 tall at z 0.10.
   - One mismatch with the rig. The rig's model-f1 rest pose (first planner row of the fixture) is
     ring 0.053 and drones 0.051-0.076, so the drones rest level with the ring. The twin rests the
     drones 0.08 above the ring origin. So the twin's rest rod starts from a different elevation,
     and its creep geometry at breakaway differs. The same bias as the legacy worlds, but here the
     world is meant to be a twin.
   - The rig world also carries the legacy params.py ring inertia (2.733e-2 / 5.45e-2 against the
     world's 2.227e-2 / 4.431e-2). That is Wesley's call (params.py), but the twin is then not a
     twin in yaw or roll.
7. **Legacy unchanged: holds.**
   - The generator's defaults are byte-identical on six legacy worlds (tested).
   - Every new parameter defaults to linear, the placeholder or 0.09. `betaflight_communication.py`
     and the Tejen bridges are untouched.
   - The pinning is implicit, though. No legacy launch or config names `sim_thrust_map: linear`, and
     nothing refuses a rig world on the linear map (or the reverse): `config_mismatches` is not
     called from `checked_config` (`run_experiment.py:812-835`).
8. **Batch runner isolation.**
   - Per slot, the domain (40+n), the partition, the bridge network, the solver and code dirs and
     the staged results are separate. Only `active` containers are removed on Ctrl-C, and fetch
     removes only its own host dir. All of this is sound.
   - Shared write: the source tree. `~/mdc` is a single RW bind mount in every container
     (`lab_run.sh`). `pick_slots` deliberately runs next to slots that are already up
     (`lab_batch.py:104-113`), and `main` then runs `lab_sync.sh` (rsync `--delete`) and
     `lab_build.sh --ws` (`lab_batch.py:291-293`) under those live runs. Their code and install
     change mid-flight, and their manifests are then wrong.
   - Provenance gap: `laptop_git_state` lists untracked files with `--exclude-standard`
     (`lab_batch.py:127`). `tools/` is git-ignored, so new files such as `make_carry_worlds.py`,
     `thrust_fit.py`, `planner_replay.py` and `lab_batch.py` are synced and run but never enter
     `git_diff.patch`. The state is also taken before the sync, while other agents are editing the
     tree.

**Verdict: A4 and W5 are sound. Nothing in W4 or W6 has run on a rig-map world yet. The rig_twin
matrix must not be queued until items 1-3 below are fixed, or every run is a LIFT_TIMEOUT and uses
up the Gazebo budget.**

Must-fix:
1. `src/controller_quad_load/launch/mpc_quad_load_launch.py:236-251` (declare) and `:306-325`
   (pass to the controller node). Add `thrust_offset` (0.0), `thrust_offset_v_slope`, `thrust_v_ref`
   and `throttle_max` (1.0), with sim defaults that leave today's runs unchanged. Add a
   `thrust_model.py` rig branch, or refuse `thrust_ratio:=auto` when `sim_thrust_map:=rig`.
2. `tools/run_experiment.py:139-142`. Count as included only the launch files actually included
   (`IncludeLaunchDescription` / `PythonLaunchDescriptionSource` with that filename), not every
   filename in the text. Then re-run `--check` on all configs; the rig_twin ones must fail until 1
   is in. Add a test with a config arg that is only declared by a launch named in a comment.
3. `tools/run_experiment.py:812-835`. Call `check_geometry.config_mismatches(world, launch, cfg.launch_args)`
   and refuse on any mismatch unless `mis_seed`. Include the plant map: `x3_rig_drone` world ⇔
   `sim_thrust_map: rig` in both launch sections. `rig_twin_model_f1.yaml:42` (pivot 0 on a pivot
   world) then needs `mis_seed: true` or a named exemption.
4. `configs/experiments/rig_twin_*.yaml`: `payload_rest_z: 0.02`. Also a written decision on
   `attach_z` for model_f1 (rig 0.0 against twin 0.02); Wesley's call.
5. `tools/remote/lab_batch.py:291-293`. Refuse to sync or build while any `wesley_slot*` container
   is up, or sync each batch into its own tree (`~/mdc_slots/tree_<batch>`, `rsync --link-dest`)
   and mount that. Take `laptop_git_state` after the sync, and at `:127` include the ignored files
   the sync ships (`git ls-files --others --ignored --exclude-standard` over the synced paths, or
   hash the synced tree).
6. `src/simulation_communication/simulation_communication/payload_betaflight_comm.py:115,143`.
   Under `thrust_map rig`, warn at arm if `_got_pack_v` is still False (the RViz launch was not
   given `sim_thrust_map:=rig`). `run_experiment` should refuse a config whose two launch sections
   disagree on `sim_thrust_map`.

Should-fix:
1. `mpc_quad_load_launch.py:272`. An empty `sim_thrust_offset` raises an IndexError. Also check
   that there is one value or exactly num_drones values.
2. `planner_node.py:426`. The slot-offset message shares the fleet manager's latched
   `/fleet/manager_status`. The manager's next status replaces it on the panel, and a late joiner
   gets whichever came last. Latch it on the planner's own phase or status topic.
3. `planner_node.py:562`. Also log the pinned (continuous) q, or a flip flag, so the CSV shows
   the fix.
4. `tools/sil/scenario.py`: a `plant: thrust_map: rig` scenario keeps `drone_mass` 0.64 unless it
   is set. Default it to 0.55 under rig, or refuse.
5. W5 rest pose: measure the rig drones' rest centre height against the ring (fixture:
   0.051-0.076 against 0.053) and decide whether the twin's drone collision box or rest z should
   follow it before W7 claims a reproduction.
6. The first rig-map Gazebo run: one free x3_rig_drone hover as the plant check (item 2), before
   the matrix.

CRITIC (phase 2): A4 and W5 are correct. The rig twin cannot fly as configured. The sim launch
drops the tracker's offset, slope and cap, and the undeclared-arg check misses it through
comment-named launch files. The batch runner re-syncs the shared tree under live slots.

### Response (phase 2)

All six must-fixes are in. Nothing was flown.

| # | Fix |
|---|---|
| 1 | `mpc_quad_load_launch.py` declares `thrust_offset` (0.0), `thrust_offset_v_slope` (0.0), `thrust_v_ref` (23.5), `throttle_max` (0.6, the node default, so today's sim runs are unchanged; not 1.0) and passes them to every tracker. `sim_thrust_map:=rig` with `thrust_ratio:=auto` or `thrust_offset` ≤ 0 is refused at launch; no rig branch in `thrust_model.py` (the rig map is typed, as on the rig launch). |
| 2 | `declared_launch_args` follows only string constants inside a `PythonLaunchDescriptionSource(...)` call. No controller_quad_load launch includes another, so the mpc chain now declares only its own args. All configs re-checked. |
| 3 | `checked_config` calls `config_mismatches` (unless `mis_seed`) and refuses any mismatch not named under a new `geometry_exempt: {key: reason}`. `load_ixx`/`load_izz` are warned, not refused: only `params.py` sets them (Wesley's call). The thrust map: a world with `x3_rig_drone` models must fly `sim_thrust_map: rig`, any other world `linear`, and both launch sections must agree. `rig_twin_model_f1` exempts `pivot_offset_z` by name; `attach_circle_n3_fast_l045/l055` and `rig_lift_n4_even_floor_timeout` exempt `cable_len` (their deliberate variable). |
| 4 | `payload_rest_z: 0.02` in all six rig_twin configs. model_f1 `attach_z` stays 0.02 (the twin's mid-plane origin) pending Wesley's word; the rig's 0.0 is referenced to its own ring body. |
| 5 | `lab_batch.py` refuses to sync or build while any `wesley_slot*` is up (re-checked just before the sync); `--no-sync --no-build` runs beside live slots on their tree. The git state is taken after the sync and includes the git-ignored code the sync ships (tools/, configs/, src/, simulation_assets/) as new-file diffs, plus one hash of the other ignored files. |
| 6 | The bridge warns once at arm when the rig map has had no pack voltage. `run_experiment` refuses configs whose two launch sections disagree on `sim_thrust_map` (with 3). |

Should-fixes: 1 (offset count: one value or `num_drones`, else refused at launch) and 4
(a rig SIL scenario without `geometry: drone_mass` is refused) are in. 2 and 3
(`planner_node.py` topic latch, pinned-q column) are left for the planner owner. 5 (rig rest
pose) needs a rig measurement. 6 (one free-drone rig-map hover first) is accepted as the first
Gazebo run on a rig world.

## Results

Lab PC runs through `tools/remote/lab_batch.py` (mdc-humble:db1c3ca, 4 cores per slot), current tree
unless stated; Group 6 is laptop SIL. Registry rows R0772-R0800.

### Group 1: W0 parity (lab against laptop)

| run | where | metric | value | baseline | verdict |
|---|---|---|---|---|---|
| R0773 | lab Gazebo, rig_lift_n4_even_floor | ring tilt peak / hold z / carried fraction | 0.49 deg / 0.601 / 1.005 | R0767 (laptop): 0.37 / 0.601 / 1.005 | PASS: same verdict, dz 0.0 cm, d tilt 0.12 deg |
| R0776 | lab SIL, carry_hover_n3 | settled z / tilt peak | 0.662 / 3.88 deg | R0775 (laptop, same tree): 0.662 / 4.91 | z PASS (0.07 mm); tilt d 1.03 deg |
| R0777 | laptop SIL repeat of R0775 | settled z / tilt peak | 0.662 / 6.39 deg | R0775: 0.662 / 4.91 | SIL noise floor: d tilt 1.47 deg on one machine |

Verdict: parity holds in Gazebo. SIL is no longer deterministic run to run (traces split 3.2-3.7 s
after start), so the 0.5 deg tilt bar is below its own noise floor; the lab SIL sits inside the
laptop spread. R0773 against R0767 changes the machine and the code together, so it is not a clean
machine parity check; the lab's own parity is then the Group 2 regression rows.

### Group 2: regression on legacy worlds (pivot 0, linear plant)

| run | config | metric | value | baseline | verdict |
|---|---|---|---|---|---|
| R0773 | rig_lift_n4_even_floor | hold z / tilt peak / solve failures | 0.601 / 0.49 deg / 0 | R0767: 0.601 / 0.37 | PASS |
| R0778 | ocp_hover_ground_creep_defaults | hold z / tilt peak / solve failures | 0.600 / 0.41 deg / 0 | R0721: 0.600 / 0.61 | PASS |
| R0779 | ocp_hover_ground_creep_n4_3915 | hold z / tilt peak / settled tilt | 0.603 / 2.47 deg / 0.98 deg | R0729: 0.600 / 1.70 / 0.64 | PASS on criteria; tilt higher, not attributed (R0729 predates the 155ee56 pretension) |

### Group 3: twin reproduction of RIG-0930-model-f1 (rig_twin_model_f1, pivot 0 in the planner)

| run | tree | solve failures | rods at hand-over (m) | hand-over elev | carried fraction | hold z | heave p-p | hold tilt mean | ref_age_frac | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| RIG-0930-model-f1 | rig, 155ee56 | 25 | 0.587-0.613 | 54-66 deg | 0.78 | 0.31-0.35 | 14 cm | 6.5 deg | n/a | baseline |
| R0781 | 155ee56 planner + tracker (W4 plant, rig world overlaid) | 0 | 0.583-0.584 | 54.8 deg | 0.996 | 0.495 | 1.2 cm | 1.24 deg | n/a (old log) | not reproduced |
| R0782 | current (W8, W12, A4; pivot 0) | 0 | 0.583-0.584 | 54.7 deg | 0.996 | 0.495 | 1.2 cm | 1.24 deg | 0.0 | not reproduced, = R0781 |

R0780 (current tree) is void: the planner compiled its OCP inside the run and starved Gazebo into a
pose-timeout abort before lift. Verdict: the twin reproduces the measured rod length and hand-over
elevation (pivot 4 cm below the centre) but not the failures or the deficit. Both are expected
from W2/W3: the failure trigger (ring yaw 180 with a 30 deg slot offset) is absent from the twin,
and its plant is the fitted law, so it carries 1.0 of the ring by construction. Line stopped
(two runs, same answer).

### Group 4: fixed twin (pivot_offset_z -0.04, rig plant, current tree)

| run | config | solve failures | rods at hand-over | hold z | heave p-p | hold tilt mean | carried fraction | ref_age_frac | baseline | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| R0783 | rig_twin_hover_fixed | 0 | 0.550 | 0.510 | 1.2 cm | 1.22 deg | 0.996 | 0.0 | R0782 (pivot 0): 0 / 0.583 / 0.495 / 1.2 cm / 1.24 | PASS |
| R0784 | rig_twin_hover_fixed (repeat) | 0 | 0.550 | 0.510 | 1.2 cm | 1.21 deg | 0.996 | 0.0 | R0783 | PASS |
| R0785 | rig_twin_hover_fixed_zki | - | - | - | - | - | - | - | R0783 | VOID (tracker solver compiled in the run, pose-timeout abort) |
| R0786 | rig_twin_hover_3915 | 0 | 0.550 | 0.504 | 0.5 cm | 1.41 deg | 0.996 | 0.0 | R0783 | PASS |
| R0788 | rig_twin_hover_fixed_zki (re-fly of R0785, solvers prebuilt in the slot) | 0 | 0.550 | 0.500 | 1.5 cm | 1.22 deg | 0.996 | 0.0 | R0783 | PASS; \|z_bias\| max 0.013 |
| R0789 | rig_twin_orbit_slow (r 0.5 at 0.125 m/s, net on) | 0 | 0.550 | 0.500 | 1.5 cm | 1.18 deg (lap 1.22) | 0.996 | 0.0 | R0788 | PASS; lap \|z err\| max 1.3 cm, xy err mean 3.1 cm (max 4.8) |

Verdict: with the pivot model the planner reads the true 0.55 m rods and holds within 1.0 cm of
target, 0 failures, both repeats identical. The hand-over elevation is 55 deg in every twin run
(pivot on or off), outside the rig card's 45 +- 5 bar; the 1.2-1.4 deg standing tilt comes from
the per-drone plant offsets (0.181-0.187) against one tracker offset with no integral.

### Group 5: rig-plant check and ring-mass mismatch neighbours (freeze on)

Every slot now builds the planner and tracker solvers before the timed run (`lab_batch.py`
`Job.warm_script`); no run below compiled in flight. The freeze has no launch arg (it is the only
pretension code path), so the freeze-off arms could not be flown. R0787 and R0790 are void
(infrastructure) and were re-flown as R0792 and R0793.

| run | config | check | value | expected / baseline | verdict |
|---|---|---|---|---|---|
| R0792 | rig_twin_free_hover_plant (drone 0 released at the magnet, 0.55 kg with rod, hover 0.6 m, 20 s) | model throttle (thr_out - thr_off) mean / sd at 23.2 V | 0.2743 / 0.0075 | law 0.2787 + (a_0 0.181 - tracker 0.185) = 0.2747 | PASS (-0.0004 against 0.01) |

| run | world ring / planner | breakaway (freeze) | carried fraction (true ring) | hold z err, z_ki 0 | ring vz up peak | planned rod tension peak / hold | rod force est. in hold (m g cf / 4 sin elev) | hold elev | solve failures | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| R0783 | 0.86 / 0.86 | none (full pull, ring +0.001) | 0.997 | +1.0 cm | 0.151 m/s | 2.90 / 2.79 N | 2.68 N | 51.6 deg | 0 | baseline |
| R0793 | 0.70 / 0.86 | 97 % of the planned pull, held there | 0.995 | +16.4 cm | 0.169 m/s | 2.82 / 2.70 N | 2.28 N | 48.4 deg | 0 | FAIL: hold z bar; fleet abort on LAND (below) |
| R0791 | 1.00 / 0.86 | none (full pull, ring +0.000) | 0.996 | -14.5 cm | 0.134 m/s | 2.97 / 2.91 N | 3.00 N | 54.6 deg | 0 | FAIL: hold z bar (expected with z_ki 0) |

Verdict: the rig plant and tracker agree in closed loop to 0.0004. A +-0.16 kg ring error moves
the hold by -14.5 / +16.4 cm with z_ki 0 and nothing else (tilt 1.2-1.3 deg, heave 1.2 cm, 0
failures), so z_ki carries it on the rig. In the light-ring arm the freeze fires at 97 %, too late
to change anything. R0793 then aborted on LAND: after "load down, rods slack" (all four drones had hovered wide,
radii 0.595/0.601/0.603/0.604 vs R0783 0.572-0.580) drone 0 tipped 4 -> 22 -> 68 deg at
49.25-49.75 s because the rigid rod reaches the ring before the drone reaches the floor. The envelope fault then disarmed the fleet with
the ring already down.


### Group 6: SIL determinism and the A4 SIL arm (laptop)

Cause of the R0775/R0777 split: the planner was never inside the SIL lockstep. The bench waited
only for the trackers (~9 ms wall per 20 ms step) while a planner solve takes ~21 ms (max 55), so
each reference landed on whichever step was current and the planner's 10 Hz ticks fell on
irregular sim times (8.24, 8.68, 9.04, ...). Not the cause: `solve_budget_s` (0 in every sim
launch, default 0), the fallback and `ref_age` (sim time), `_refresh_model_cap` (inactive with
`thrust_offset` 0). Three smaller sources showed once that was fixed: a node's timer could fire
on /clock before that step's poses arrived; the planner's warm start depended on how many ticks
it ran while the trackers booted; event times came from float `sim_t - t_ready`.

Fix (bench and planner, sim time only): the planner publishes `/planner/tick [t, period]` after
every tick when `use_sim_time` is on, and a bench step on which a tick is due waits for it; the
bench publishes poses before /clock; the payload pose is withheld until the scenario clock starts,
which starts on a planner tick; events and the run length count steps; the tracker's model-cap and
cable-static freshness timers use the node clock. Tests: `tools/test/test_sil_lockstep.py`,
`tools/test/test_sil_load_yaw.py`, `src/controller_load_mpc/test/test_sil_tick_marker.py`.

Bisect (scratch results tree, not registered; each row a pair of identical runs):

| bench | max \|dz\| | max \|d tilt\| | first split |
|---|---|---|---|
| base (as R0775/R0777) | 9.68 mm | 1.26 deg | refs 0.14 s, plant 1.04 s |
| + planner in the lockstep | 3.03 mm | 1.05 deg | plant 3.20 s |
| + poses before /clock | 0.38 mm | 0.042 deg | tracker u 1e-12 at 1.24 s (planner warm start) |
| + payload withheld until start on a tick | 0 | 0 | none (logged ref0 lagged a step at ticks: drained) |

| run | config | metric | value | baseline | verdict |
|---|---|---|---|---|---|
| R0796 | carry_hover_n3, final bench | settled z / tilt peak / settled tilt / solve failures | 0.6643 / 5.05 deg / 0.833 deg / 0 | R0775/R0777: 0.662 / 4.91-6.39 | deterministic reference for this tree |
| R0797 | carry_hover_n3, repeat | max \|dz\| / max \|d tilt\| vs R0796 | 0.0 mm / 0.0 deg (every column, events and metrics identical) | bars 1 mm / 0.01 deg | PASS |
| R0798 | carry_hover_n3_yaw180, A4 on | solve failures / fed-q sign flips / tilt peak / settled z | 0 / 24 / 5.04 deg / 0.6642 | R0796 | PASS, not discriminating |
| R0800 | carry_hover_n3_yaw180, A4 off | solve failures / fed-q sign flips / tilt peak / settled z | 0 / 21 / 5.04 deg / 0.6643 | R0798 | 0 failures without the fix |

R0794/R0795 (float-timed events, LAND 16.000 vs 16.020) are superseded and R0799 is void (PYTHONPATH).
Events now fire on the exact step (ARM at 1.000, was 1.020 by float luck), so this tree's
carry_hover_n3 baseline is R0796, not R0775. A4 off was a scratch `sitecustomize` that replaces
`quat_same_hemisphere`; R0800 differs from R0798, so the switch took effect. Verdict: SIL is
deterministic again. At yaw 180 with the drones on their slots, the ring crosses +-180 about 20
times without a solve failure either way, so SIL cannot show A4's effect: the replay trigger also
needs the 30 deg slot offset (W2), which the bench cannot spawn (plant and planner share
`attach_azimuths_deg`).

### Twin on the flown code (fba2d7a, laptop)

One headless run per arm on the laptop, the exact rig code (fba2d7a, clean tree), against the lab twins.
Bars: 0 failures, hold z within 1 cm, tilt within 0.3 deg, heave within 0.5 cm; orbit xy within +10 %.
Hand-over is the planner's pivot reading (log / per drone); hold metrics over the last 20 s.

| run | config | baseline | failures | rods at hand-over | hand-over (log / d0-d3) | hold z | heave p-p | tilt mean / max | carried | ref_age | RTF | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R0810 | rig_twin_hover_fixed | R0783 / R0784 | 0 | 0.550 x4 | 55.4 / 57.5, 56.2, 55.6, 55.4 | 0.5103 | 1.17 cm | 1.21 / 1.40 | 0.997 | 0.0 | 0.32 | PASS |
| R0811 | rig_twin_hover_fixed (repeat) | R0783 / R0784 | 0 | 0.550 x4 | 55.5 / 57.4, 56.3, 55.5, 55.5 | 0.5103 | 1.18 cm | 1.22 / 1.54 | 0.997 | 0.0 | 0.32 | PASS |
| (base) | R0783 / R0784 | | 0 | 0.550 x4 | 55.4-55.5 / 57.4, 56.4, 55.6, 55.4 | 0.5102 / 0.5103 | 1.19 / 1.17 cm | 1.22 / 1.43 | 0.997 | 0.0 | 0.27 / 0.28 (lab) | |
| R0812 | rig_twin_hover_3915 | R0786 | - | - | - | - | - | - | - | - | - | VOID (drone 2 pose timeout 0.26 s before TAKEOFF, no compile) |
| R0813 | rig_twin_hover_3915 (re-fly of R0812) | R0786 | 0 | 0.550 x4 | 55.5 / 57.5, 56.2, 55.5, 55.5 | 0.5038 | 0.45 cm | 1.42 / 1.79 | 0.996 | 0.0 | 0.30 | PASS |
| (base) | R0786 | | 0 | 0.550 x4 | 55.4 / 57.5, 56.3, 55.4, 55.8 | 0.5038 | 0.49 cm | 1.41 / 1.77 | 0.996 | 0.0 | 0.30 | |
| R0814 | rig_twin_hover_fixed_zki | R0788 | 0 | 0.550 x4 | 55.6 / 57.5, 56.2, 55.6, 55.6 | 0.4996 | 1.48 cm | 1.21 / 1.68 | 0.997 | 0.0 | 0.28 | PASS (planner solver rebuilt in the run, ~80 s wall, lift t 25.1 vs 17.9) |
| R0816 | rig_twin_hover_fixed_zki (clean re-fly of R0814) | R0788 | 0 | 0.550 x4 | 55.5 / 57.8, 56.4, 55.7, 55.5 | 0.4996 | 1.46 cm | 1.21 / 1.52 | 0.998 | 0.0 | 0.31 | hold PASS; FAIL: fleet disarm on LAND |
| (base) | R0788 | | 0 | 0.550 x4 | 55.6 / 57.5, 56.1, 55.6, 55.6 | 0.4996 | 1.46 cm | 1.22 / 1.50 | 0.998 | 0.0 | 0.24 | |
| R0815 | rig_twin_orbit_slow | R0789 | 0 | 0.550 x4 | 55.5 / 57.6, 56.3, 55.7, 55.5 | 0.5001 | 1.50 cm | lap 1.22 / 1.69 | 0.996 | 0.0 | 0.30 | PASS |
| (base) | R0789 | | 0 | 0.550 x4 | 55.5 / 57.5, 56.3, 55.7, 55.5 | 0.5001 | 1.47 cm | lap 1.22 / 1.54 | 0.996 | 0.0 | 0.25 | |

| orbit lap (same window) | tilt mean | \|z err\| max | xy err mean | xy err max |
|---|---|---|---|---|
| R0789 (base) | 1.22 deg | 1.27 cm | 2.91 cm | 4.80 cm |
| R0815 | 1.22 deg | 1.28 cm | 2.95 cm | 5.08 cm (+6 %) |

R0816 LAND:

| t (s) | event |
|---|---|
| 50.45 | ring down, planner "load down, rods slack" |
| 50.45-51.8 | drone 0 pushed out by its rod (radius 0.58 -> 0.80 m), rate commands saturate (roll -1.0, yaw +1.0), tilt 20 -> 29 -> 74 deg |
| 51.86 | envelope fault at 77.9 deg, fleet disarm (ring on the floor; drones 1-3 <= 3.6 deg) |

Same mechanism as R0793 LAND, now on the nominal twin: 1 of 11 nominal twin landings (the other 10 landed
with every drone <= 21 deg). Not load or timing: ring true climb speed 0.119 m/s (R0788 0.118); the planner's
vz estimate peaked at 0.28 in both laptop z_ki runs (R0788 0.151), an estimate only; stale-reference
warnings at baseline level.

Verdict: hold, hand-over and orbit on fba2d7a reproduce the lab twins (hold z within 0.1 mm, tilt mean
within 0.01 deg, heave within 0.04 cm, 0 failures, orbit xy max +6 %); the rigid-rod LAND tip is the open risk.

## Status before the 1 Oct rig visit

Verified in sim (runs above):

| item | evidence | value |
|---|---|---|
| A2 pivot model | R0783, R0784, R0786, R0788, R0789 | rods 0.550 (0.583 without), hold within 1.0 cm on z_ki 0 |
| A1 + A2 + A3 together, 0 failures, ref age 0 | twin runs R0781-R0793 | 0 failures; ref_age_frac 0.0 in R0782-R0789 and R0791; R0793 0.0028 (during the LAND abort); R0781 has no ref_age column |
| A3 bookkeeping on the affine plant | R0792 | model throttle 0.2743 against 0.2747 |
| net on (H2 twin) and circle (C1 twin) | R0788, R0789 | \|z_bias\| 0.013; lap tilt 1.22 deg, \|z err\| 1.3 cm, xy 3.1 cm |
| ring mass +-0.16 kg, z_ki 0 | R0791, R0793 | height only: -14.5 / +16.4 cm, tilt 1.2-1.3 deg, 0 failures |
| SIL deterministic again | R0796 / R0797 | 0.0 mm, 0.0 deg apart; this tree's baseline is R0796 |

Only the rig can tell: whether A1/A4 remove the failures (neither sim reproduces the trigger: ring yaw 180
with the 30 deg slot offset; R0798 / R0800 give 0 failures with A4 on and off); the ~0.8 force deficit
and its height dependence (D1, D2; the twin carries 0.996 by construction); A3 closed-loop takeoff (D1).

Open risks: hand-over at 55 deg in every twin run (creep lead, drones 10-16 deg ahead of the arc from
liftoff), the creep fix does not move it (R0801-R0809, below); the H1 row is now 50-60 on the pivot reading. LAND with rigid rods can tip a drone that hovers wide of the ring (R0793) and disarm the fleet
after the ring is down. The freeze has no off switch, so the W11 freeze-off arms are unflown. sil_smoke
still fails its peak cable acceleration bar (10.56 against 8.0) until it is re-baselined.

### Creep fix

Lift-off margin path starts the sweep at the measured mean elevation, clipped to [spawn, target]
(`creep_controller.py` `_arc_creep`, +9; 4 unit tests). Critic: sound, no must-fix items. Should-fixes not
applied in this change: (1) timeout path still sweeps from spawn (rigid-rod stall case), (2) `meas` from the
live ring pose rather than `arc_anchor`, (3) `dtheta` uses `cable_len` not `_rod(i)` (pre-existing). Unflown:
needs one floor-start twin run on the lab PC (latch below 55 deg, measured within ~6 deg of the reference).

### Creep fix runs

Lab PC, one batch of six (R0801-R0806) and a re-fly of its three voids on three slots (R0807-R0809).
Hand-over elevation is the planner's pivot reading at the creep-to-planner switch (log line, then per
drone); lift-off is the first drone 5 cm above its spawn; tilt and vz are the drone peaks in the 2 s after it.

| run | config | hand-over (log / d0-d3) | TAKEOFF to hand-over | lift-off to hand-over | sweep start | lift-off tilt / vz | failures | hold z | heave p-p | hold tilt mean | carried | baseline | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R0807 | rig_twin_hover_fixed | 55.6 / 57.7, 56.5, 55.7, 55.6 | 3.56 s | 2.82 s | 8.5 deg | 4.09 deg / 0.52 m/s | 0 | 0.510 | 1.2 cm | 1.22 deg | 0.997 | R0783: 55.4 / 3.87 / 3.11 / 2.1 / 3.66, 0.49 / 0 / 0.510 / 1.2 / 1.22 | FAIL hand-over; hold PASS |
| R0808 | rig_twin_hover_fixed (repeat) | 55.7 / 57.5, 56.4, 55.7, 55.7 | 3.57 s | 2.82 s | 8.4 deg | 3.71 deg / 0.54 m/s | 0 | 0.510 | 1.2 cm | 1.22 deg | 0.997 | R0784: 55.5 / 3.92 / 3.16 / 2.1 / 4.34, 0.58 / 0 / 0.510 / 1.2 / 1.21 | FAIL hand-over; hold PASS |
| R0803 | rig_twin_hover_3915 | 55.7 / 57.6, 56.3, 55.7, 55.7 | 3.63 s | 2.88 s | 7.1 deg | 3.76 deg / 0.59 m/s | 0 | 0.504 | 0.5 cm | 1.42 deg | 0.163 (metric) | R0786: 55.4 / 6.95 / 3.17 / 2.1 / 4.09, 0.52 / 0 / 0.504 / 0.5 / 1.41 | FAIL hand-over; hold PASS |
| R0804 | rig_twin_orbit_slow | 55.5 / 57.6, 56.3, 55.5, 55.6 | 3.64 s | 2.91 s | 7.0 deg | 4.01 deg / 0.60 m/s | 0 | 0.500 | 1.4 cm | 1.18 deg | 0.996 | R0789: 55.5 / 3.91 / 3.17 / 2.1 / 4.81, 0.61 / 0 / 0.500 / 1.5 / 1.18 | FAIL hand-over; hold PASS |
| R0809 | rig_lift_n4_even_floor | 45.7 / 45.7, 45.8, 45.8, 45.8 | 2.94 s | 2.37 s | 16.2 deg | 1.39 deg / 0.45 m/s | 0 | 0.601 | 2.5 cm | 0.02 deg | 1.005 | R0773: 45.8 / 3.38 / 2.80 / 7.5 / 1.70, 0.45 / 0 / 0.601 / 3.6 / 0.02 | PASS |
| R0806 | ocp_hover_ground_creep_defaults | 45.5 / 45.7, 45.5, 45.7 | 3.01 s | 2.41 s | 14.0 deg | 1.66 deg / 0.44 m/s | 0 | 0.600 | 0.2 cm | 0.03 deg | 1.005 | R0778: 45.7 / 3.35 / 2.75 / 7.6 / 1.56, 0.52 / 0 / 0.600 / 0.2 / 0.04 | PASS |

R0801, R0802 and R0805 are void: wall-clock pose timeouts with six 4-core slots on the 24-core PC
(Gazebo at ~0.25 real time); re-flown as R0807-R0809. R0803's carried fraction is a pairing artifact of
the metric (its tracker throttles and hold z equal R0786's).

Creep trace (planner log, once a second):

| run | at lift-off: ref / measured | +1 s: ref / measured | ref at target: ref / measured | latch |
|---|---|---|---|---|
| R0783 (base) | 2.1 / not logged | 12.5 / 24.8-26.7 | 45.0 / 56.8-59.1 | 55.4 |
| R0807 (fix) | 8.5 / 8.5 | 23.1 / 34.3-38.1 | 45.0 / 55.9-57.9 | 55.6 |

Verdict: FAIL on the hand-over bar (55.5-55.7 against 45 +- 5, unchanged from 55.4-55.5). The fix does what
it says (the sweep now starts where the drones are, 7-8.5 deg, and hand-over comes 0.3 s sooner), but the
12-15 deg lead builds in the first second after lift-off, while the drones leave the 0.10 m vertical lead at
0.5-0.6 m/s, so it was not a start-angle offset. Hold metrics, failures and the lift-off jerk are unchanged
(hold z within 0.1 mm, tilt within 0.02 deg, lift-off tilt 3.7-4.1 against 3.7-4.8). The legacy floor
worlds still hand over at 45.5-45.7. Not iterated.

### Creep fix: removed (Wesley, 1 Oct)

The sweep-start change did not move the hand-over angle (R0803-R0808: 55.5-55.7 deg, against 54.7-55.5 before). It was reverted the same session (house rule: code that does not beat its baseline is deleted). The lead comes from the lift-off pop (0.5-0.6 m/s off the 0.10 m vertical lead). After the visit, the options are: cap the climb rate at lift-off, or make the latch two-sided (a gate change).

## Reviewer (30 Sep / 1 Oct)

Verdict: WEAK.

Supported: the twin numbers (recomputed from the run logs, all within 10 %); SIL bit-identity (R0796 /
R0797); the rig-plant closed-loop check R0792; each arm changes one variable.

Weak: single runs (H2/H3/C1 twins R0788, R0786, R0789; the fba2d7a re-flies R0813, R0816, R0815 are one
more run each, on the laptop); the HEAD-code gap is closed: R0810-R0816 fly the exact rig code fba2d7a and
match the lab twins (hold z within 0.1 mm, tilt within 0.01 deg, orbit xy max +6 %), but R0816 tipped drone 0
on LAND and disarmed the fleet (R0793 mechanism, 1 of 11 nominal twin landings); the scope is twin-only (fitted law, carried ~1.0 by construction), so nothing in sim
predicts the rig's force shortfall.

Not supported as first written (corrected above): replay "25 -> 0 failures" (it is 14 -> 0 for arm g on
the 155ee56 planner, trigger inferred); carried fraction "rising with height inside every flight" (across
flights only); "ref_age_frac 0.0 in every twin run R0781-R0793"; R0793 as drone 0 alone hovering wide
(all four did) with tip angles 8 -> 42 -> 76; R0779 settled tilt 0.86 (0.98).

Action before the rig: none blocking; the rig flights are exploratory data (D1/D2) plus hovers with stop
rules.

**1 Oct, before the visit (Wesley):** D1 (free hover without the rod) and the magnet checks are dropped from tonight's session. Whether rod drag biased the thrust fit stays open; D2 still measures the carried fraction against height.

**1 Oct, at the rig:** the QUAD3 airframe is out, so tonight flies three drones (QUAD1/2/4, mocap bodies 11/12/14), even ring on plates 0/4/8. H3 (1/3/5/9) is dropped. No 3-drone rig twin was flown first (the rig stack was up on the laptop and the lab SSH needed re-approval).
