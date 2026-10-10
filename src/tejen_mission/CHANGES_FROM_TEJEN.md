# Changes from Te-Jen's code

Upstream: `drone_cage_control` (Te-Jen Yu), branch `SwungPayload2026_Collaborative`.
Rule (Wesley): his code may be edited here, but every edit is logged in this file: the file,
what changed, why, and our commit. Files we add inside his packages are listed too.

Path map (his -> ours): `src/drone_communication` -> `src/tejen_mission`;
`src/dynamic_planner_ros` -> `src/tejen_dynamic_planner`;
`tools/standalone_dynamic_planner_cpp` -> `src/tejen_dynamic_planner/standalone`;
`src/controller_mpc_payload` -> `src/tejen_mpc`; `src/utility_objects` -> `src/tejen_utility_objects`.
His `tools/sim_test/` and `tools/irl_test/` scripts live at the same paths in our tree.

| sync | his commit | date |
|---|---|---|
| fork | a7524bae | 25 Sep 2026 |
| sync 1 | 11a0683f | 10 Oct 2026 |

## (a) Edits our fork carries against his code (since a7524bae)

### Renames and paths (no behaviour change)

| file(s) | what | why | commit |
|---|---|---|---|
| all four packages | package, Python module, resource, include and import names renamed per the path map | his `drone_communication`, `controller_mpc_payload`, `utility_objects` collide with packages already in this workspace | 418a4fd |
| `tejen_dynamic_planner/CMakeLists.txt` | `DYNAMIC_PLANNER_CORE_ROOT` default `../../tools/standalone_dynamic_planner_cpp` -> `standalone` | the standalone planner core is vendored inside the package | 418a4fd |
| tests (`test_m2a_*`, `test_m2b_*`, `test_m2c_integration_audit`, `test_m2d_*`, `test_octopus_multi_partial_fallback`, `test_p3_sim_time`), `launch/c1f6_system_test`, `launch/m2b_b0/b1_single_attachment`, `tejen_dynamic_planner/README.md` | `simulation_assets/X` -> `simulation_assets/tejen/X`; `tools/standalone_dynamic_planner_cpp` -> `src/tejen_dynamic_planner/standalone` | his worlds and models live under `simulation_assets/tejen/` here | 418a4fd |
| M2/C1E/C1F launches and tests | executables `betaflight_communication`, `motion_capture_emulator` and `betaflight_linear_simulation_launch.py` -> `tejen_`-prefixed names in `simulation_communication` | his sim nodes sit next to ours in that package | 418a4fd |
| `tools/sim_test/*` (`apply_m2a_paused_bootstrap_revert.py`, `run_m2a_attachment_foundations.sh`, `run_m2b_b0/b1.sh`, `run_sim_test.py`, `validate_m2b_*.sh`, `test/test_xy_disturbance_sim_contract.py`) | same asset paths | as above | 639d20e |
| `package.xml` | `exec_depend mpc_planner` | `ring_bridge` imports our `LoadTrajectory` | 418a4fd, bbce103 |
| `setup.py` | `ring_bridge` entry point | our node (below) | 418a4fd |

### Behaviour

| file | what | why | commit |
|---|---|---|---|
| `motion_capture_publisher_irl.py`, `motion_capture_publisher_node.py`, `motion_capture_publisher_node_global_coords.py` | a malformed or undecodable UDP packet is skipped (`continue`) instead of `return None` | one bad packet left the receive loop and every pose went stale mid-flight | f8c3c66 |
| `online_join_planner.py` | `publish_magnet_command` re-sends the current command every 1 s | a single on-change publish was lost when the magnet manager subscribed late (R0619: the pickup ON never arrived); both magnet managers are idempotent | 418a4fd |
| `online_join_planner.py` | optional `m2d_transit_owner_topic` ('' = off): after the takeoff hover settles, a vehicle waits until it holds the transit token | one C++ transit at a time: his safety checker threw on a moving peer's short window and simultaneous joins stalled (T0009, T0012-T0014) | 418a4fd |
| `mission_definitions.py` | rejoin descent and its hold: magnet ON (was OFF), in both missions that rejoin | the weld happens at the end of that descent (Wesley 25 Sep) | 418a4fd |
| `m2d_sequential_supervision.py`, `m2d_fleet_supervisor.py` | `simultaneous` (default false): every unattached vehicle holds mission permission at once | testing all four at once (Wesley 26 Sep) | 418a4fd |
| `launch/m2d_four_drone_sequential.launch.py` | `simultaneous` argument; planners get `m2d_transit_owner_topic`; his sim bridges get `rate_ki` (`M2_RATE_KI`, else `SIM_RATE_KI`, else 5) and `rate_source` (`M2_RATE_SOURCE`, default imu) with one IMU bridge per X3 and a world check | sim rate-loop I-term (T0011); the M2 ring rock was his bridges differencing unstamped poses under CPU load (R0709-R0720) | 418a4fd, dc65350, fdd41f4, c651388, 6a2d748 |
| `launch/m2c_four_drone_ground.launch.py` | same `M2_RATE_SOURCE` handling and IMU bridges | as above | 6a2d748 |
| `m2c_ground_spawn.py` | `--imu-system` (default on) adds one world Imu system; `M2C_TETHER_ORIGIN_BODY_Z_M` overrides the tether origin (default 0.01) | gyro rate loop; body-centre pivot test copy | 418a4fd, c651388, 6a2d748 |
| `launch/m2b_b0_single_attachment.launch.py`, `launch/m2b_b1_single_attachment.launch.py` | `launch_guard` refuses a world without the Imu system | the bridges default to the gyro | 6a2d748 |
| `tejen_dynamic_planner/launch/c1e_commissioning_sim.launch.py` | `rate_source` argument (default imu) and an IMU bridge | as above | 6a2d748 |
| `tejen_dynamic_planner/launch/c1f6_moving_rendezvous_backend.launch.py` | `use_sim_time`, `state_topic`, `object_attached_topic`, `extra_params_file` arguments | run his backend namespaced in our partner stack | 418a4fd |
| `launch/c1f6_system_test.launch.py`, `launch/m2c_vehicle_controller.launch.py` | his MPC `thrust_ratio` 82.7; `m2c_vehicle_controller` gets `elrs_output_topic` (default `ELRSCommand`) | our linear sim plant (x3 motorConstant 0.62e-6 at 0.643 kg); route his ELRS through our per-drone mux at the M2 hand-over | 418a4fd |
| `c1f2_handoff.py` | padding past the cached C++ window is a stopped hold (zero velocity and acceleration) | padding with the terminal velocity froze the position while commanding ~0.2 m/s and the recovery check rejected every fresh reference (M2 join stall T0023, T0025, T0032, T0033) | df62cd7 |
| `magnet_attachment_manager.py` | gz attach/detach sent 4 times 0.3 s apart off the executor; a newer command cancels older repeats | a one-shot `gz topic -p` was lost under load (R0615: ATTACH logged, object never moved) | 418a4fd |
| `pendulum_state_publisher.py` | rates over the measured pose period, updated only when a new pose arrives; object velocity only over a positive dt | the timer and the throttled sim clock ran out of step with the poses: 26-530 rad/s spikes (R0597, R0609, R0614) | 418a4fd |
| `simulation_test_supervisor.py` | takeoff counts as observed in any phase past WAIT_FOR_TAKEOFF; optional `start_gate_topic` holds the schedule until our load planner reports running; an unanswered ARM is re-sent after 2 s; readiness log every 5 s | R0637 (TAKEOFF_NOT_OBSERVED, disarm mid-air after an air hand-over); partner mode starts his mission at our release; R0606 (ARM_TIMEOUT) | 418a4fd |
| `tejen_mpc/main.py` | `pose_timeout_s`, `thrust_ratio` (default 22), `rates_centre_deg`, `rates_max_deg`, `rates_expo`, `cable_length` (default 0.531) as parameters, defaults unchanged; `object_mass_kg` gain schedule on the object-attached flag (0 = off) | our sim plant (kT ~83; R0620 pose stall at 0.25 real time); the 0.1 kg partner object | 418a4fd, 49aa340 |
| `tools/sim_test/run_m2d_sequential_attachment.sh` | `M2D_SIMULTANEOUS`, `M2D_HANDOVER` (dynamic 0.86 kg ring, `ELRSCommand_tejen`, operator hold), `M2_RATE_SOURCE` (imu default), `M2D_SOURCE_X3`, `~/ros2_humble` sourced first, observer start grace 5 s and ready wait 60 s, GZ resource path adds `simulation_assets/tejen` | M2 hand-over to our stack; gyro rate loop; startup races on a loaded laptop (T0011, T0013) | 639d20e |
| `tools/sim_test/run_m2c_ground.sh` | `M2_RATE_SOURCE` and `--imu-system` | as above | 639d20e |
| `tools/sim_test/run.sh`, `validate_m2a_runtime_increment.sh`, `README.md` | source `~/ros2_humble` when ROS is not sourced; repo root from the script's location; an M2 hand-over bench section | this machine's ROS is the source build | 639d20e, bbce103 |

### Files we added inside his packages

| file | what | commit |
|---|---|---|
| `tejen_mission/ring_bridge.py` | our carried ring and carriers published in his fake-world formats (subclass of `FakeCooperativeTransportWorld`) | 418a4fd, bbce103 |
| `launch/partner_mission.launch.py`, `config/partner_backend_overrides.yaml` | his M1 mission flown by our drone 3 in our sim | 418a4fd, c06c21e, 49aa340, 6310a7e, bbce103 |
| `test/test_c1f2_padding_deadlock.py` | the stopped-hold padding | df62cd7 |
| `test/test_m2_rate_source_imu_contract.py` | the gyro rate source on his M2C/M2D path | c651388, 6a2d748 |
| `test/test_ring_bridge_markers.py` | `ring_bridge` keeps his fixed marker builder | sync 1 |
| `tools/sim_test/`: `desk_*.py`, `drive_m2_handover.py`, `drive_m2d_headless.py`, `make_m2_bench_world.py`, `stage1_torque_test.py` | our drivers and desk checks around his M2 | 639d20e and later |

## (b) Syncs

### Sync 1, 10 Oct 2026: a7524bae..11a0683f (17 commits, 25 Sep to 8 Oct)

Classification: `docs/tejen_sync_2026-10-08.md`. Wesley 10 Oct: "Start now, sim only"; branch `tejen-sync`,
nothing to the rig tree before 14 Oct. Every opt-in feature keeps his default (`attachment_policy`
legacy, `pickup_alignment_mode` legacy, `target_alignment_mode` legacy, `trajectory_type` circle,
`cooperative_shared_start_skew_s` 0).

Taken:

| what | his commits | files (ours) |
|---|---|---|
| marker aliasing fix: the label marker no longer moves the published attachment pose up 0.12 m (reaches `ring_bridge`, which inherits `make_markers`/`timer_callback`) | fef23390 | `fake_cooperative_transport_world.py`, `test/test_m1_ground_ring.py` |
| C++ planner: identical cooperative updates no longer bump the world version; rolling rechecks from the post-snapshot time with coverage errors; opt-in bounded peer start skew in the backend | 82fbff07, 906ee3fb | `standalone/src/world_snapshot.cpp`, `standalone/src/receding_horizon_planner.cpp`, `standalone/tests/test_world_snapshot.cpp`, `standalone/tests/test_recheck_commit.cpp`, `src/transfer_backend_node.cpp` |
| planner, M1: measured-tip pickup proof and stalled-pickup recovery, near-surface candidates, ground ring, takeoff ground datum and capped LAND target, opt-in drop/ring measured-tip trim | 91c0f63a, 8d6a9695, f3829d57, fef23390, 611dc3d2, c7e88c65, 11a0683f | `online_join_planner.py`, new `m1_pickup_proof.py`, `m1_pickup_recovery.py`, `m1_ground_ring.py`, `measured_tip_alignment.py`, `payload_geometry.py`, `fake_cooperative_transport_world.py` (`measured_stationary`) |
| planner, M2: gate profiles, 5 deg takeoff swing, landing phases no longer fault on permission revocation, tip trim, same-plate heartbeats keep the retry count, `measured_contact_15deg` policy | d9c69415, 5065bee9, cb5c96e8, 906ee3fb, e89ef7ca, 0138b9e1 | `online_join_planner.py`, `m2b_attachment_mission.py`, new `m2_irl_attachment_policy.py`, `attachment_detection.py`, `m2_attachment_observer.py`, `mission_definitions.py` |
| M2D progressive policy (operator wait instead of abort, `fresh` evidence), opt-in | e89ef7ca | `m2d_fleet_supervisor.py`, `m2d_sequential_supervision.py`, `launch/m2d_four_drone_sequential.launch.py` |
| IRL (rig only when launched): two/four-vehicle identity, freshness classes, ring geometry, mocap ring publisher and duplicate-ID check, body rates no longer rounded to 0.1 rad/s in the IRL publisher, calibration YAMLs, four-drone YAML, M1 overlays and ground-ring launch | d9c69415 to 11a0683f | `m2_irl_*.py`, `motion_capture_publisher_irl.py`, `config/irl_commissioning.yaml`, `config/m2_irl_two_drone.yaml`, new `config/m2_irl_four_drone.yaml`, `config/m1_*.yaml`, `launch/m1_irl_*.launch.py`, `launch/m2_irl_two_drone.launch.py`, `tejen_dynamic_planner/launch/c1f_irl_m1_virtual_ring_backend.launch.py` |
| tools | as above | `tools/irl_test/99-elrs-m2.rules`, new `check_m1_ground_ring.py`, `latest_m2_irl_log.sh`, `run_m1_ground_ring.sh`, `run_m2_irl_four_drone.sh`; `tools/sim_test/run_m1_irl_virtual_ring.sh`, `run_m2d_sequential_attachment.sh` |
| tests: 13 new and 7 modified Python, 2 modified C++ (with the C++ row) | all | `test/` |

Three-way merge (base a7524bae, ours = rig-prep c2279cf, his 11a0683f, names mapped): clean with our
edits kept in `online_join_planner.py` (1 s magnet resend, transit token), `mission_definitions.py`
(rejoin magnet ON), `motion_capture_publisher_irl.py` (receive-loop fix), `fake_cooperative_transport_world.py`
and two tests. Conflicts, resolved by hand:

| file | hunk | resolution |
|---|---|---|
| `m2d_fleet_supervisor.py` | core constructor arguments: our `simultaneous` vs his `progressive_policy` | both arguments passed |
| `m2d_sequential_supervision.py` | `__init__` signature and fields | both (`simultaneous`, `progressive_policy`, `waiting_reason`); our simultaneous branch stays before his progressive landing wait, so with both off the path is his legacy one |
| `launch/m2d_four_drone_sequential.launch.py` (3) | imports; his `generate_launch_description` split into `_launch_setup` (OpaqueFunction); the argument list | his OpaqueFunction structure kept; our `_rate_source`/`_require_imu_world` helpers before it; our `simultaneous` argument and the IMU-world check declared next to his `attachment_policy`; node parameters unchanged (launch dump identical before and after, default and `simultaneous:=true`) |
| `tools/sim_test/run_m2d_sequential_attachment.sh` | the launch command's last line | both `attachment_policy:=` and our `simultaneous:=` |

Edits made in this sync to his files:

| file | what | why |
|---|---|---|
| `test/test_m2_irl_io_launch_contract.py` | `tools/standalone_dynamic_planner_cpp` -> `src/tejen_dynamic_planner/standalone` in `test_rolling_commitment_recheck_has_future_coverage_or_rejects_late` | path map |
| `launch/partner_mission.launch.py` (ours) | docstring: his ready point was ~0.18 m above the plate before this sync, 0.12 m of it from the marker bug | the fix moves it |
| `test/test_m2_rate_source_imu_contract.py` (ours) | expands his `_launch_setup` OpaqueFunction (policy legacy) to find the nodes; picks the IMU-world check by function name | his M2D launch now builds its nodes in an OpaqueFunction |

Skipped:

| what | why |
|---|---|
| his `drone_visualisation` (M2 fleet panel, `m2d_four_drone.rviz`, his arm panel) | we run our own `arm_panel`; his 6 RViz/panel contract tests fail here for that reason (5 failed before this sync; `test_m2_irl_ring_calibration::test_existing_rviz_assignment_topic_shows_numbered_plates_and_frozen_pair` is new) |
| `quad_payload_integral_dynamics_ocp.json` (repo root) | generated; `tejen_mpc` builds into the acados cache |
| docs, command sheets, `REPO_INDEX.md`, `backup/` moves, `.code-workspace` | not code |
| combined-stack changes (one mocap owner, `ELRSCommand_tejen` remap on the rig) | a later, separate step (sync plan step 4) |
| his MPC, `elrs_interface*`, thrust map | unchanged upstream in this range |

Tests after the sync (`python3 -m pytest src/tejen_mission/test`): 834 passed, 12 failed; before it
(rig-prep) 721 passed, 14 failed. His own tree at 11a0683f fails the same 5 stale lab-YAML and B2-obstacle
assertions (`test_irl_m1_attachment_retry::test_irl_config_uses_one_detach_dwell_without_extra_thresholds`,
`test_irl_m1_virtual_ring_contract::test_one_lab_yaml_owns_pickup_geometry_for_planner_and_attachment_manager`,
`::test_measured_irl_pickup_object_geometry_is_consistent_across_m1_stack`,
`test_m2b_b2_cpp_transit_contract::test_b2_launch_has_one_optional_cpp_static_obstacle_while_b1_default_remains_cpp_off`,
`::test_b2_obstacle_blocks_direct_corridor_without_occupying_start_or_capture_goal`). The other 7 read files we do
not carry (his RViz configs and panel, his arm panel, his repo `.gitignore`). C++: 15/15 ROS tests,
standalone 19/20: `test_suspended_system` exceeds its 240 s ctest timeout on this laptop, before this sync too
(the rig-prep build's `static_overflight` scenario alone ran past 239 s).

## 10 Oct 2026 (late): the combined M1 subtest on the rig twin (branch m1-subtest; Wesley 10 Oct)

| file | change | why |
|---|---|---|
| `launch/partner_mission.launch.py` (ours) | `geometry:=legacy` (default, every value as before) or `rig_twin`: ring radius 0.225, rods 0.55 m at 60 deg, plate pitch diameter 0.45, ring collision diameter 0.51, `magnet_drop_below_quad` 0.59, MPC `cable_length` 0.55, `thrust_ratio` 22 with his thrust-ratio feedback on (his rig values, irl_commissioning.yaml), vehicle mass 0.55; the nodes are built in an OpaqueFunction | his mission on the rig-twin attach world (QUAD4 0.55 kg through the rig thrust map, where 82.7 for the linear plant cannot lift it off) |
| `mission_types.py` | `MissionPhase` `LIFT_PROOF`, `RING_ATTACHED`, `TAKEN_BACK`; a `PhaseSpec` magnet command may be `NONE` (the planner sends nothing, the manager keeps its last state) | the contact attach below; on the rig his object magnet is QUAD4's one coil, so after the attach or a take-back he must send neither ON nor OFF |
| `mission_definitions.py` | `pickup_delivery_mission`, `join_only_mission`, `mission_for_mode` take `contact_attach` (default false): `ATTACH_READY` without `handoff_ready`; `LIFT_PROOF` (rate-controlled, attachment point, magnet ON), `RING_ATTACHED` (target-relative, magnet NONE) and `TAKEN_BACK` (stationary, magnet NONE) join the phase map; m2b refuses it. Off: the phase maps are as before | combined M1 contact attach (card 2026-10-10_combined_m1_contact rev 4, contract `docs/design/m1_contact_contract.md`): our manager confirms the catch, his hand-back flag is not wired |
| `online_join_planner.py` | `contact_attach` (default false; refused with `ground_ring_attach_trial` or m2b) and, only when on, `contact_topic` /magnet/contact, `ring_attached_topic` /magnet/object_attached, `give_up_topic` /join_planner/give_up (latched, repeated 1 Hz), `take_back_topic` /drone_3/take_back, `mux_state_topic` /drone_3/mux_state, `proof_rise_max` 0.01, `proof_rise_rate` 0.02, `contact_max_failures` 3, `contact_give_up_s` 30, `contact_match_xy_threshold` 0.025, `contact_match_xy_velocity_threshold` 0.08, `contact_abort_xy_threshold` 0.05, `contact_abort_xy_velocity_threshold` 0.15. Ahead of every other guard (`update_contact_attach`): `/magnet/contact` true in DESCEND or ATTACH_READY -> `LIFT_PROOF` (rises plate-relative at the rate to the cap, no lead); contact false -> DESCEND again from the lifted offset, failures + 1; 3 failures or 30 s (node clock) since the first DESCEND start -> give-up -> `TAKEN_BACK`; `/magnet/object_attached` true in TRANSIT_TO_REATTACH..LIFT_PROOF -> `RING_ATTACHED` (holds the offset it had, plate-relative, until the mux reads ours, then the body where it is); `/drone_3/take_back` true in any phase -> `TAKEN_BACK` (holds the last reference's first point). Both terminal: LAND, take-back, contact and alignment changes are ignored, no magnet command. The rejoin's match and abort gates take the contact values (his `match_xy_*` also gate the drop settle, which keeps his). String aliases for the three phases | as above; R1473: after our LAND his ATTACH_READY abort guard restarted the approach and handed off again |
| `simulation_test_supervisor.py` | optional `pickup_release_state_topic` ('' = off; `pickup_release_detach_topic` /pickup/detach, `pickup_release_attach_topic` /pickup/attach): `PickupReleaseGate` repeats detach at 1 Hz until the joint's state reads `detached` for 1 s; after 5 s without any state, one attach forces a fresh pair; readiness (so ARM and TAKEOFF) waits for it, the startup timeout still counts | the ball's spawn weld is a 1 m lever and a one-shot release was lost (R0678); gz publishes the joint state only on a change, so a detach landed before the bridge listened leaves no echo |
| `ring_bridge.py` (ours) | `full_pose_attachment` (default false): the published attachment point is ring position + R(q) @ (plate offset + `bridge_attach_z_m`) with the ring's measured attitude, as our manager's weld point; yaw-only until an attitude arrives | a tilted ring moves the plate by r sin(tilt): 8 deg is 3 cm at r 0.225 |
| `launch/partner_mission.launch.py` (ours) | `plate` and `carrier_azimuths_deg` default to the geometry's (legacy 9 and 330/90/210, rig_twin 3 and 270/30/150; his plates count counter-clockwise, his 3 = +y = rig plate 9); `contact_attach` (default false, rig_twin only): planner `contact_attach`, `final_attach_offset_z` 0, `attach_ready_target_lead_time` 0; ring_bridge `bridge_attach_z_m` 0.032 (plate top 0.02 + the 12 mm tip sphere) and `full_pose_attachment`; the bag adds the contact topics. `confirm_pickup_release` (default false): bridges `/pickup/detachable_joint_state` (gz.msgs.StringMsg), drops the one-shot `gz topic` detach and sets the supervisor's release gate. Defaults: the launch dump is identical to before for legacy (default, `record`, explicit plate/azimuths) | the combined M1 contact attach; the geometry profile owns his plate and carriers (critic round 3, item 5) |
| `test/test_m1_contact_attach.py`, `test/test_m1_contact_targets.py` (ours, new), `test/test_partner_mission_geometry.py` (ours) | his phase transitions under `contact_attach` and with it off; the full-pose target equal to our manager's weld point on a tilted ring; the release gate; the launch switches; the geometry test evaluates parameters as the launch passes them | tests first (card section 8) |
| `launch/partner_mission.launch.py` (ours) | `contact_lead`, `contact_dz`, `contact_dx` launch args (default 0) for his hold's lead, target height and x offset under `contact_attach` | the card's neighbour runs (`_lead030`, `_high`, `_low`, `_offset`) |

## 10 Oct 2026 (night): the full M1 on the rig twin (branch m1-full; Wesley 10 Oct)

His code is unchanged; the launch below is ours.

| file | change | why |
|---|---|---|
| `launch/partner_mission.launch.py` (ours) | `airborne_start` (`auto` default: true when `start_gate_topic` is `/partner/release`; `true` / `false` override): where his kT feedback flies (rig_twin), his MPC gets `thrust_ratio_ukf_min_height_above_start` 0.0 (his default 0.20). The subtest (gate `/payload/trajectory_state`) and the legacy geometry pass nothing new | his MPC re-latches its kT estimator's start height on every disarmed tick, so it arms at QUAD4's step-out point (1.85 m in R1503) and learns kT only 0.20 m above it; his mission rose at most 0.14 m before the pickup descent: the estimator never started (`waiting_for_height` for the whole flight), kT stayed at 22 (the subtest converges to ~21.1 from the mat) and his MPC, with no z integral, flew 11 cm under his reference (settle 0.847 vs 0.961, descent end 0.588 vs 0.701): the tip went to the floor at the ball |
| `test/test_partner_mission_airborne_kt.py` (ours, new) | his `run_full_model_thrust_ratio_ukf` executed from his source on R1503's heights (default gate: never starts; 0.0: starts at his takeoff; mat start as before) and the launch switch | as above |
| `launch/partner_mission.launch.py` (ours, m1-final) | the airborne gate is -10.0, not 0.0; the test adds a descent below the arm height that keeps learning (0.0 pauses it there) | his gate is re-checked on every tick, not only at the start: with 0.0 the estimator started at his takeoff but paused (`waiting_for_height`, kT frozen at 22) as soon as QUAD4 went below the height it armed at (~1.85 m), his whole pickup at ~0.7 m (R1505). Real-life note for Tejen: on the rig too his gate stops kT learning whenever the vehicle flies below its arm height, not only before the first climb |

## 10 Oct 2026 (night): the proof lift and the pickup retry (branch m1-final; Wesley 22:20)

| file | change | why |
|---|---|---|
| `online_join_planner.py` | `LIFT_PROOF` (`contact_attach`): on entry the tip's measured offset from the contact target is latched (his measured magnet tip and ring_bridge's attachment point, both fresh within `input_timeout_s`); the lift runs from the offset commanded at that moment (no reference step) to `proof_rise_max` above that measured height, xy as before. A stale or missing measured tip, or `use_measured_magnet_tip` off, keeps the lift from the target (logged once). A failed proof re-presses from the lift reference, as before | R1498 (`contact_dz` -0.01): the tip rests on the plate 1 cm above a low target, so the 1 cm lift from the target only brought it back to the plate; three caught contacts, each proof timed out (plate rise peaks 2.6 / 0.8 / -0.1 mm against the 2.5 mm bar), then his give-up |
| `online_join_planner.py` | `pickup_retry_enabled` also takes effect with `contact_attach` (before: `measured_tip_tiered` only). His retry as it is: no object after `pickup_retry_wait_s` in MAGNET_ATTACH_WAIT (or the object lost in the lift), object magnet OFF, a vertical-only rise until the face clears the object by `pickup_retry_clearance_m`, then APPROACH_ABOVE_PICKUP over the object where it now is and his descent again. Under `contact_attach` his spent attempt budget (`await_operator`, reached after the third failure's rise) is the contact give-up: `/join_planner/give_up` true, `TAKEN_BACK` holding where the rise stopped | R1493, R1497 stalled in MAGNET_ATTACH_WAIT until our LAND (~150 s); his retry was unreachable with the legacy alignment our launch flies. (The cause first given here, no weld, was wrong: see the next rows) |
| `launch/partner_mission.launch.py` (ours) | with `contact_attach`: `pickup_retry_enabled` true and `pickup_retry_clearance_m` 0.10; his wait (10 s), rise speed, rise cap (0.20 m) and 3 attempts kept | a pickup with no ball: rise ~0.10 m, re-centre, descend again; two retries, then the give-up and our take-back |
| `test/test_m1_contact_attach.py`, `test/test_partner_mission_geometry.py` (ours) | the lift from a low target ends `proof_rise_max` above the measured contact; the nominal target within 1 mm of before; no reference step entering from the descent; the stale fallback, logged once; the retry on the rig-twin ball geometry: stall, rise, re-centre on the pushed ball, descend again, a catch on the retry, the give-up after the third failure, retry off as before; the launch values | tests for both |
| `online_join_planner.py` | `pickup_hold_on_attach` (default false): in DESCEND_TO_PICKUP, the object flag ends the descent: MAGNET_ATTACH_WAIT holding the body so the tip stays at its measured height (the commanded xy; the commanded point if the measured tip is stale), through his candidate hold (`m1_candidate_hold_reference`), so his lift starts from that hold | every stall (R1493, R1497, R1500 on e097a94, R1507 on m1-final): the ball DID weld (his object flag at ~36 s, tip at z 0.175 m over the ball centre at 0.08), and his descent ran ~2 s on to its profile end, pushing the welded ball into the floor; the rigid twin rod buckled sideways (R1507: tip (1.04, 1.74), body (1.01, 1.50) against the target (1.0, 1.6)), so his `pickup_lift_xy_tolerance` (0.08) never held. Good runs (R1491, R1506) buckle the same way and recover by chance (R1506: 16.6 s). Only where the object flag is the weld itself: his rig manager (`magnet_attachment_manager_irl`) raises it on proximity (tip within 0.12 m, 0.8 s), which would end the descent above the object; his rig path (`measured_tip_tiered` with `pickup_proof_enabled`) already holds at a measured near-surface candidate (`pickup_candidate_max_clearance_m` 5 mm) and proves co-motion in the lift |
| `online_join_planner.py` | MAGNET_ATTACH_WAIT under `contact_attach`: the contact-wait stall counts "not aligned" (attached but the tip off target, or nothing attached), not only "nothing attached"; the same release (object magnet OFF drops the object), rise, re-approach where it lies, same 3-attempt budget and give-up. Without `contact_attach` his rule as before | his retry was blocked while the flag was true, so the attached-but-bent stall never ended |
| `launch/partner_mission.launch.py` (ours) | with `contact_attach`: `pickup_hold_on_attach` true | the twin's object flag is his sim manager's weld (tip within 0.10 m of the ball centre) |
| `test/test_m1_pickup_recovery.py` (his) | the source check for the contact-wait stall's predicate reads the new line (his `not self.object_attached` kept when `contact_attach` is off) | the predicate changed on purpose |
| `test/test_m1_contact_attach.py`, `test/test_partner_mission_geometry.py` (ours) | his own tracking errors and a weld model as his sim manager's: the descent ends on the weld and holds the tip where it caught, the lift starts from that hold; without the hold the descent pushes on to the target; attached but bent: the ball dropped, rise, re-centre, a catch on the retry; bent on every catch: the give-up after the third; without `contact_attach` an attached tip off target is his wait; the launch value | tests |
