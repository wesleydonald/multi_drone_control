# Integrating Tejen's code: decisions, interfaces, and who flies what

Gitignored. Working notes for Wesley. Last updated 2026-09-25 evening.

## 1. What was brought in (decided: we fork once and own the copy)

Source: `~/tejen/drone_cage_control`, branch `SwungPayload2026_Collaborative`, commit `a7524bae`
(Te-Jen Yu). Each package has a `PROVENANCE.txt`.

| his package | ours | what it is |
|---|---|---|
| `drone_communication` | `src/tejen_mission` | mission planner (`online_join_planner`), magnet manager, fake ring, supervisors, IRL nodes, launches, configs, 771 tests |
| `controller_mpc_payload` | `src/tejen_mpc` | his acados payload-pendulum MPC that flies the drone |
| `dynamic_planner_ros` + `tools/standalone_dynamic_planner_cpp` | `src/tejen_dynamic_planner` (planner source in `standalone/`) | C++ obstacle-avoiding transfer planner (needs GLPK: `sudo apt install libglpk-dev`) |
| `utility_objects` (his version) | `src/tejen_utility_objects` | his CallbackManager, arm permission, logger, visualisation |
| `interfaces` msgs | added to our `interfaces` | `CommittedTrajectory`, `CubicTrajectoryPiece`, `PlanPath.srv` (the shared messages were identical) |
| his worlds/models | `simulation_assets/tejen/` | 21 files: his worlds and everything they include |
| `tools/sim_test`, `tools/irl_test` | same paths | his runner scripts |

Why renamed: his `drone_communication` and `controller_mpc_payload` clash with ours (our
`drone_communication` is the ELRS interface; our `controller_mpc_payload` is an older copy of his
MPC used by the attach launch's optional approach MPC). Nothing of ours was changed by the import
except `interfaces` (two messages and one service added) and `.gitignore`.

Test status in our copy: 750 of 771 of his tests pass. His own repo fails 9 of them. The other 12
check files in packages we share with his repo, where his repo has additions ours lacks (item 3 below).

## 2. What each side does in the demo

```
ring flies a circle (ours) ──► DETACH drone k (ours) ──► drone k held at its detach point (ours)
      ──► HANDOFF OUT ──► pick up object, drop it into the basket on the moving ring,
                          fly back, align over plate, descend (Tejen: tejen_mission + tejen_mpc)
      ──► HANDOFF IN at ATTACH_READY ──► magnet weld, OCP resize n→n+1, circle continues (ours)
```

His M1 mission already does the middle part in sim and in the cage, against a virtual moving ring.
His M2 (four drones join a grounded ring, tilt out, hand the whole system over) is an alternative
takeoff to our creep start; it ends in a fleet-level handover to our OCP.

## 3. Mismatches to resolve (in the order they will bite)

1. **Rate model.** His MPC assumes Betaflight rates centre 100, max 100 deg/s (linear); our sim bridge
   and our trackers assume 70 / 670 deg/s with expo 0.5. The leaving drone is one of Wesley's
   airframes, so his MPC's rate parameters must match what those flight controllers run.
   **Decided (Wesley 2026-09-25): Tejen's profile is correct and is now used everywhere**: our
   trackers (new params `rates_centre_deg` / `rates_max_deg` / `rates_expo`, default 100 / 100 / 0.5),
   the velocity loop, all three sim Betaflight nodes, the SIL plant and stand-in. Linear 100 deg/s
   at full stick (was 670 deg/s max in our model). Every earlier sim result is being re-checked.
2. **Thrust gain.** His kT is hard-coded (22 rig / 44 his sim). On our linearised sim it is 83.1;
   on the rig our airframes use 24. Make it a parameter.
3. **Shared sim packages.** His launches need `pendulum_state_publisher` (missing in ours) and
   parameterised/namespaced versions of `betaflight_communication` and `motion_capture_emulator`
   (topics, telemetry). Ours carry the linear thrust mapping and today's rate-timing fix, so his
   additions get merged INTO ours, not copied over them. His RViz panel additions (ADVANCE button,
   fleet panel) live in the shared `drone_visualisation`: bring over later if wanted.
4. **Start state.** His missions start on the ground. The demo needs a "take over a drone hovering at
   its detach point, trailing its cable" entry state.
5. **The ring as his basket.** He expects ring pose/twist, a rolling 10 s CommittedTrajectory for the
   ring, and each other drone's state + CommittedTrajectory (obstacle avoidance). Our ring flies an
   analytic circle, so a 10 s commitment is easy; a bridge node publishes our ring and drones in his
   formats. His drop target is the ring centre (the basket).
6. **Rejoin plate.** His rejoin targets plate 0; ours is the gap the drone left (plate 3). Config.
7. **Magnet at ATTACH_READY.** **Decided (Wesley): his descent ends with the magnet ON**, so the
   weld happens at the end of his descent and our side detects it and folds the drone in.

## 4. Authority protocol (Wesley: fine; he is taking it to Tejen)

One authority per drone at every instant; the fleet manager and the abort path cover every drone
whatever its authority.

| state | who flies drone k | entered when | left when |
|---|---|---|---|
| FLEET | our tracker, OCP references | takeoff / weld | DETACH command |
| RELEASED_HOLD | our tracker, hold at the detach point (cable trailing) | magnet OFF + OCP resize n→n−1 | Tejen's MPC is live (armed, fresh reference whose first sample is within 5 cm of the hold) |
| MISSION | Tejen's MPC, his planner | our mux switches drone k's ELRS stream to his MPC | his `handoff_ready` (= ATTACH_READY) |
| REJOIN | his MPC until the weld, then ours: weld detection, OCP resize n→n+1 | his descent with the magnet ON | weld confirmed; our tracker live; his MPC idles |

The switch is a per-drone ELRS stream select (our `elrs_mux`, one per drone) that only moves when the
receiving controller is already publishing live commands, so there is never a zero-throttle gap.
Tejen's design note rejects per-weld switching for his M2 fleet handover; for leave/rejoin a
per-drone switch at exactly two instants is unavoidable, so this needs his agreement.
Topics on the boundary (proposal): `/drone_k/authority` (String: FLEET | RELEASED_HOLD | MISSION |
REJOIN, published by our dissipative node), his `/join_planner/handoff_ready`, his
`<ns>/m2d/mission_permission` reused as the go signal, `/drone_k/magnet` for the weld.

## 5. Plan (validate each step before the next)

1. GLPK installed, C++ planner builds, his tests green in our tree.
2. His M1 in our sim against his fake ring, on our X3 model and linear plant (kT 83.1, rates matched).
3. Bridge node: our real ring + drones in his formats; his pickup and drop target the carried ring.
4. Authority protocol above: detach → his mission → rejoin, in one flight.
5. Object mass landing in the basket: height integral and trim absorb it (bounded).
6. His M2 (four-drone ground join, tilt, handover) as an alternative takeoff.

## 6. Changes to his code for the M2 handover (2026-09-26, T0007-T0013)

- (tried and reverted: `endsInStoppedHold` default tolerance 1e-9 -> 1e-6; T0013 stalled the same way)
- `tejen_betaflight_communication` (sim bridge): rate loop via `rate_pid.RatePid`, params
  `rate_kp` 0.5 / `rate_ki` (0 = the old P-only loop; default changed in step 2b, below),
  `rate_i_limit`, `rate_i_min_u`. `m2d_four_drone_sequential.launch.py` reads `M2_RATE_KI` for his
  four bridges.
- `m2c_ground_spawn.py`: `M2C_TETHER_ORIGIN_BODY_Z_M` env (default 0.01) for pivot test copies.
- `run_m2d_sequential_attachment.sh`: `M2D_SOURCE_X3` override; simultaneous branch honours the
  operator hold (prints "M2D SUCCESS HOLD" and waits for the handover driver); 5 s grace before
  the observer process-group check (a setsid race failed two starts).
Why: his tether pivots 0.04 m below the X3 body; loaded at 45 deg the lever needs ~0.08 N m, which
the P-only sim rate loop cannot hold (spins at ~257 deg/s); a real Betaflight holds it with its
I-term. Small-scale evidence S0001-S0010 in results/registry.csv.
- 2026-09-26 (step 2b, Wesley's word 2026-09-26 in decisions.md and the GOALS planning answers
  ): `m2d_four_drone_sequential.launch.py` `M2_RATE_KI` default 0 -> `SIM_RATE_KI`, else 10.0;
  all four sim bridges now default `rate_ki` to `SIM_RATE_KI` (10.0). `SIM_RATE_KI=0` gives the old
  P-only loop everywhere; `M2_RATE_KI` overrides it for the m2d bridges only.

- 2026-09-27: `m2d_four_drone_sequential.launch.py` rate_ki fallback 10.0 -> 5.0 and tejen_betaflight_communication default 5 (T0017 ring rock at ki 10 near the 3.2 Hz PI zero; T0018 clean at 5).
- 2026-09-27: partner_mission.launch.py object magnet backend gz_cli -> both, with ros_gz bridges for /pickup/attach|detach (the one-shot gz CLI detach dropped: the ball stayed welded after DROP in R0678 and missed the net).
- 2026-09-27: FORK BASE = Tejen's drone_cage_control commit a7524bae (verified: unmodified m2_irl_readiness.py matches a7524bae; d9c6941 "m2-irl: consolidate gate profiles" [2026-09-25 22:08] is NOT in the fork). Plan (Wesley/Tejen): when Tejen finishes his state-machine renumbering, merge his a7524bae..HEAD (path drone_communication -> tejen_mission) in one go, re-apply this section's changes, run his tests + one headless M2 and one M1 before committing to integration.
- 2026-09-27: d9c6941 impact check (read-only): behaviour-neutral for our sims. online_join_planner.py changes: c1f2_prepare_settle_acceleration_mps2 (0.50) replaces c1f2_handoff_acceleration_tolerance_mps2 (0.50) in the prepare gate; m2b_pendulum_timeout_s (0.35) replaces m2b_evidence_timeout_s (0.35) for pendulum freshness; m2b_ring_pose_timeout_s default 0.25 -> 0.50 (our M2D launch already sets 0.50; M1 does not run M2B); ring-pose freshness uses that timeout instead of input_timeout_s (0.50 = 0.50); new takeoff-settle diagnostics/CSV columns (log only; our tools do not parse his CSV). The m2_irl_* files are rig-only. Caveat for the merge: overrides of c1f2_handoff_acceleration_tolerance_mps2 / m2b_evidence_timeout_s would no longer reach those gates (none today).
- 2026-09-27: (reverted) m2b_ground_spawn.py universal-tether acceptance for the T0021 test; universal tethers broke his flight (drone 0 at 90 deg after his join).
- 2026-09-27: tejen_betaflight_communication opt-in `rate_source` 'pose' (default, unchanged) | 'imu' + `imu_topic` (default `imu`, i.e. /drone_i/imu): the rate loop steps on each gyro sample with dt from its sim-time stamps (rate_pid.gyro_sample) instead of differencing unstamped poses over a 0.25 s window-average period; in imu mode the pose subscription is not created, warn once if armed with no gyro sample. Why: the M2 bench ring rock is load-dependent (R0707/R0708 calm at RTF 0.30, R0709 7.46 deg at RTF 0.169). Only m2_bench_io_launch.py rate_source:=imu uses it (bridges his imu_sensor, 1000 Hz, from m2_bench_world_imu.sdf = the bench world + the Imu system his M2C generator strips); his m2d launch is untouched.
- 2026-09-27: full M2 path gyro opt-in `M2_RATE_SOURCE=imu` (default `pose`, generated world and launch unchanged in effect): `run_m2d_sequential_attachment.sh` passes the new `m2c_ground_spawn.py --imu-system` (one world-level `gz-sim-imu-system` plugin after UserCommands, the same placement as `m2_bench_world_imu.sdf`; the X3/ring SDFs and every entity are byte-identical, so pose_index / target_object_id 7 hold; manifest `run_local_imu_sensors_enabled` true); `m2d_four_drone_sequential.launch.py` reads the env, sets his four bridges' `rate_source` + `imu_topic` /drone_i/imu, bridges /world/quadcopter/model/x3_i/link/X3/base_link/sensor/imu_sensor/imu -> /drone_i/imu, and refuses a world without the Imu system; `drive_m2_handover.py` logs the source. Why: the gyro removed the bench ring rock (pose 3/6, gyro 0/6). Tests: src/tejen_mission/test/test_m2_rate_source_imu_contract.py.
- 2026-09-28 (Wesley's decision in decisions.md): the gyro is the default on every sim bridge. `tejen_betaflight_communication` `rate_source` default 'pose' -> 'imu'; `m2c_ground_spawn.py` CLI adds the Imu system by default (`--no-imu-system` for pose; the Python function default stays off); `m2d_four_drone_sequential.launch.py` `M2_RATE_SOURCE` fallback pose -> imu; `m2c_four_drone_ground.launch.py` gains the same `M2_RATE_SOURCE` switch, four `/drone_i/imu` bridges and the world guard; both runners default `M2_RATE_SOURCE=imu` and pass `--no-imu-system` for pose; the legacy single-X3 `tejen_betaflight_linear_simulation_launch.py` (m2b_b0/b1, c1f6) and `c1e_commissioning_sim.launch.py` bridge the X3 gyro to `/imu` with a `rate_source` arg (default imu); `m2b_b0/b1` refuse a `world_path` without the Imu system (`simulation_communication.imu_world`). Pose on any path: `M2_RATE_SOURCE=pose` or `rate_source:=pose`. Sim only: no real launch starts a bridge.
- 2026-09-28: his three UDP mocap publishers in our fork (tejen_mission/motion_capture_publisher_irl.py, _node.py,
  _node_global_coords.py): `return None` inside the receive loop -> `continue` (+ skip undecodable packets), the same bug
  and fix as our publisher (ed7084d): one malformed packet ended the loop and every pose went stale. His m2_irl_mocap_router
  is already robust (timer poll, decode errors=replace). His own repo still has the bug in motion_capture_publisher_irl.py
  (used by irl_state_pipeline.launch.py = his M1 IRL): TELL TEJEN.
- 2026-09-29: `c1f2_handoff.advance_reference_window` pads past the cached window with a STOPPED hold (terminal position,
  zero velocity and acceleration) instead of the terminal state. Root cause of the M2 join stall (T0023/T0025/T0032/T0033):
  the C++ backend went stale mid-transit for longer than the 2 s cached window; the padding froze the position but kept
  ~0.22 m/s, and the C1F.2b recovery check (online_join_planner.py ~4430, 0.03 m / 0.10 m/s / 0.5 m/s^2) rejected every
  fresh C++ reference (which starts from the held drone at <= ~0.09 m/s) forever; the backend, believing it had authority,
  tripped its 0.05 m tube along the fixed direction to the target (the constant (-0.008, 0.056, -0.027) offset), braked,
  replanned, repeated. Windows that end stopped are unchanged. Offline repro + fix test:
  src/tejen_mission/test/test_c1f2_padding_deadlock.py. TELL TEJEN (his repo has the same padding).
- 2026-10-03 (rename, plan noble-noodling-lollipop §3): three references in `tejen_mission` to our renamed
  packages updated in place, nothing else: `tejen_mission/ring_bridge.py` imports
  `mpc_planner.load_trajectory` (was `controller_load_mpc`), `package.xml` exec_depend `mpc_planner`,
  `launch/partner_mission.launch.py` docstring `ros2 launch bringup three_attach_launch.py` (was
  `controller_quad_load`). His launches still run `drone_magnet` executables, which is why the attach
  chain stayed in `drone_magnet`.
