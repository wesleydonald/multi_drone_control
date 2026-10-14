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

Classification: `docs/tejen/tejen_sync_2026-10-08.md`. Wesley 10 Oct: "Start now, sim only"; branch `tejen-sync`,
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
| `online_join_planner.py` | `TAKEN_BACK` under `contact_attach`, entered after the drop (CLEAR_DROP_ZONE onward: TRANSIT_TO_REATTACH, APPROACH_ABOVE_TARGET, MATCH_VELOCITY, DESCEND_TO_ATTACHMENT, ATTACH_READY, LIFT_PROOF), by our take-back or his give-up: his object magnet OFF, held (re-sent by his 1 s keepalive like any command), logged once. Entered while he carries the ball (before the drop): no object-magnet command, as before | R1520 (m1c_nocatch_orbit, 6143989): he gave up at 137.19 s with the rejoin's object magnet ON; the ball lay in the ring's net and rolled toward plate 9 on the circle; at 138.3 s our step-out brought the tip within 0.102 m of it, his twin manager (0.10 m catch) welded it, and the step-out dragged the net: QUAD4 tilted 61 deg (fault LAND), then the ring 61 deg (fleet abort). Real-life note: on the rig his object magnet is QUAD4's one coil, so our take-back's OFF (once our span began) releases it as well; in the twin his object magnet is separate and stayed ON |
| `test/test_m1_contact_attach.py` (ours) | his give-up after the drop holds the object magnet OFF; our take-back from each post-drop phase does too; a take-back while he carries the ball (lift, transit, drop settle and descent) sends no object-magnet command | tests |
| `online_join_planner.py` | `LIFT_PROOF`: the lift ends `proof_rise_max` above the higher of the target offset and the latched measured contact height (was: the measured height alone); the LIFT_PROOF line logs both | R1522 (m1c_orbit_2, 6143989): on three caught contacts the tip read 5-6 mm below the target (the ring sagging under the press), so the goal ended 4-5 mm above the commanded offset; his MPC (no z integral, under load) moved the body 1.4 mm in 2.75 s, the plate rose at most 1 mm, the proofs timed out and he gave up. The target-based rise would have lifted 10 mm. The R1498 case (the tip resting above a low target) still rises from the measured contact |
| `test/test_m1_contact_attach.py` (ours) | a tip pressed below the target lifts to target + rise; the goal over (target, tip) pairs, never below the target-based lift; the log line | tests |

## 11 Oct 2026: the combined M1 contact attach on the rig (branch m1-rig, from m1-subtest e097a94)

Everything is behind switches that default off: `partner_mission.launch.py real:=false` (the default) builds the
sim graph exactly as before (the `_record_typed` OpaqueFunction only notes the typed arguments), and the
supervisor's `stop_action` defaults to his `disarm`.

| file | change | why |
|---|---|---|
| `simulation_test_supervisor.py` | `stop_action` (default `disarm`, his behaviour): `land` publishes `/join_planner/land_now` where a stop would request a disarm, with a warning naming the stop's reason | combined-stack H5: on the rig a stale input (his ball body occluded under the magnet or in the net) must never cut a flying vehicle's motors |
| `launch/partner_mission.launch.py` (ours) | `real:=true` (default false) and geometry `rig` (refused without `real`): no Gazebo node, the wall clock; his IRL values copied in per node from `irl_commissioning.yaml` (online_join_planner, magnet_attachment_manager_irl, tejen_mpc blocks; never the whole file: its root topics include our `/magnet/object_attached`), with every topic written out under `/tejen`; his planner in his IRL file order plus the pickup values of his last successful IRL pickup (`m1_ground_ring.yaml`: min_reference_z, pickup overtravel, the support allowance; `pickup_alignment_mode measured_tip_tiered`); `contact_attach` required, `contact_target_z` typed (= our calibrated weld_point_z, ring_bridge with the full ring attitude), contact 1 cm below the plate (`contact_dz` -0.01), lift cap `proof_rise_max` 0.03, transit 2.3 / 2.7 / 1.35 m, body-to-marker 0.575 (his IRL); his MPC with his IRL block, kT 22 + UKF + feedback, cable 0.531, pose timeout 0.25, `object_mass_kg` 0 (his kT feedback owns the gain, as in his IRL flights), `ELRSCommand` -> `/drone_3/ELRSCommand_tejen`, `telemetry` -> `/drone_3/telemetry`; his IRL ball manager in `/tejen` with its radio output off; his backend with `partner_backend_overrides_rig.yaml`; his supervisor only with `supervisor:=true` (manual, `stop_action land`, rig timeouts). New arguments (read only when real): `real`, `contact_target_z`, `magnet_drop_below_quad`, `proof_rise_max`, `mpc_thrust_ratio`, `mpc_cable_length`, `anchor_offset_z`, `supervisor`; a typed value wins over the rig default | the combined M1 on the rig (Wesley 10 Oct: Tejen's controller does every rig attach; our motion capture alone confirms it); one launch with ours (`real_control_launch.py mode:=attach partner_nodes:=true`) |
| `config/partner_backend_overrides_rig.yaml` (ours, new) | the sim file's carrier columns (0.15 / 0.15 / 0.30) plus his IRL object box (0.043 / 0.043 / 0.011, centre 39 mm below the marker) and his 15 deg IRL swing envelope | his backend around our real carriers with the real 60 g puck |
| `tejen_mission/partner_rig_state.py` (ours, new; `setup.py` entry point) | from our mocap (`/drone_3/motion_capture_state`, `/magnet_tip_pose`, `/pickup/motion_capture_state`) to his `/tejen/magnet_tip_pose`, `/tejen/pendulum_swing_state` (his IRL pendulum math: anchor `R(q) @ (0, 0, -0.030)`, phi/theta, 8-sample rate mean, pair age 0.20 s), `/tejen/pickup_object/pose` (PoseArray, index 0) and `/tejen/payload_world_state` | combined-stack H3: our mocap node is the only Motive owner on the combined rig, so his IRL publisher does not run |
| `test/test_partner_rig_state.py`, `test/test_partner_mission_rig.py` (ours, new); `test/test_simulation_test_supervisor.py` (`StopActionTest` appended) | the pendulum equal to his IRL publisher's own code on 200 random poses; the rig graph (no Gazebo node, wall clock, no topic of ours written, the contact target, the rig values, typed values win, refusals); `stop_action` | tests first |

## 11 Oct 2026 (night): the rig port's critic round 1 (branch m1-rig, after dd33dca)

| file | change | why |
|---|---|---|
| `online_join_planner.py` | `drop_check_puck` (contact_attach only, default false) and `drop_stuck_radius_m` (0.08): DROP_OBJECT waits its release dwell, then the object MEASURED within the radius of the measured marker is a contact give-up (TAKEN_BACK, `/join_planner/give_up`), away or not measured (warning) clears the drop zone; `measured_object_on_tip`, `update_measured_drop` | critic F9: on the rig his object flag is inferred and clears with the OFF itself, so a puck held by residual magnetism would ride into the rejoin over the ring |
| `simulation_test_supervisor.py` | `stop_action` also takes `give_up`: a stop publishes `/join_planner/give_up` (the fleet takes the vehicle back) | critic F9: land_now leads to his LANDING, whose magnet OFF drops the object over the ring |
| `launch/partner_mission.launch.py` (ours) | real:=true: `magnet_drop_below_quad` and `mpc_thrust_ratio` refused untyped (his IRL defaults are QUAD2's: body 12, marker 22); `supervisor:=true` refused on the rig (its stop would be `give_up` once the critic has seen it); his planner gets `drop_check_puck` true | critic F8, F9 |
| `test/test_m1_contact_attach.py`, `test/test_partner_mission_rig.py`, `test/test_simulation_test_supervisor.py` (ours / appended) | the measured drop (dwell, stuck = give-up, away or stale = clear, switch off = his), the rig refusals, the give-up stop | tests first |

## 11 Oct 2026: m1-final (d8c1b65) merged into m1-rig

| file | change | why |
|---|---|---|
| `launch/partner_mission.launch.py` (ours) | the rig path (real:=true) keeps his IRL pickup values: his retry comes from his `m1_measured_tip_pickup.yaml` (`pickup_retry_enabled` true there); `pickup_hold_on_attach` and the airborne kT gate stay twin-only (his rig object flag is inferred proximity, not the gz weld; the rig full M1 has no card) | the twin's reasons do not hold on the rig |
| `online_join_planner.py` | his LIFT_PROOF goal (f29763c, d8c1b65: `proof_rise_max` above the higher of the target and the measured contact) applies to the rig unchanged: with the rig's 1 cm-low aim and the tip resting on the plate the lift ends 0.03 above the measured contact; his TAKEN_BACK after the drop holds his object magnet OFF | as merged |
| (ours) `drone_magnet/magnet_gate.py` | his OFF is held only from the pickup to the drop: it also passes in his pickup phases (his retry releases there, also for an object attached but not aligned) and after the drop (his TAKEN_BACK's OFF, d8c1b65); the welded start (`start_ours`) hands the coil to him at `/partner/release` | m1-final's retry and full M1 next to critic F9 |

## 11 Oct 2026: the QUAD4 tilt cut card (branch m1-tiltcut3 on m1-rig ead6afe; card docs/experiments/2026-10-11_quad4_tilt_cut.md rev 3, rig-port critic F6, tilt-cut critic round 2)

| file | change | why |
|---|---|---|
| `online_join_planner.py` | `taken_back_self_land` (contact_attach only, default false) with `taken_back_mux_wait_s` 1.0, `taken_back_clear_radius_m` 1.2, `taken_back_step_speed` 0.2, `taken_back_land_z` 0.20: in TAKEN_BACK, if `/drone_3/mux_state` does not read ours (or latched / cut) 1 s after entering it, a two-leg reference (radially out from the ring centre to 1.2 m at its height, then down at `landing_descent_speed` to `taken_back_land_z`) and, once down for `landing_min_time_before_disarm_s`, his `landing_disarm_command` on his drone command topic; the object magnet is TAKEN_BACK's own (d8c1b65: OFF after the drop, no command while he carries the ball). Off: TAKEN_BACK holds where it is, as before | our warm drone-3 tracker latches faults while it does not fly QUAD4, so a take-back can never switch the mux; TAKEN_BACK ignores LAND and his MPC has no LAND of its own, so QUAD4 hovered on his stream until the operator's DISARM |
| `launch/partner_mission.launch.py` (ours) | `taken_back_self_land` launch argument (default false), passed to the planner only under `contact_attach` | as above |
| `online_join_planner.py` | `mux_state_callback` (contact_attach; replaces the lambda that only stored the state): `/drone_3/mux_state` reading `cut` (the fleet has disarmed the vehicle in the air: its tilt, our switch `partner_tilt_cut`, or our cut at LANDED, `partner_stuck_guard`) publishes his `landing_disarm_command` once and goes to `LANDED_DISARMED`. The state only reads `cut` with one of our switches on | tilt-cut critic round 2 M-E: his MPC kept streaming armed after the cut, so a relaunched mux (which starts uncut) would forward it armed and the motors could restart on the floor |
| `test/test_taken_back_self_land.py` (ours, new) | the wait, the fly-out and descent, one disarm; the object magnet OFF after the drop and no command while he carries the ball; a switched or cut mux holds; off holds; the launch switch; the mux's `cut`: one disarm, the mission ended | tests first |

## 11 Oct 2026 (evening): the rig port's critic round 2 (branch m1-harden, from m1-comove 5e70f54; rig card docs/experiments/2026-10-21_combined_m1_rig.md, A7)

| file | change | why |
|---|---|---|
| `online_join_planner.py` | `DROP_PUCK_WAIT_S` (2.0 s, a module constant): under `drop_check_puck` (contact_attach only, default false; the rig launch sets it) an object or marker not measured after the release dwell no longer clears the drop zone. DROP_OBJECT waits up to 2 s more for fresh poses (the object away: clears; on the marker: the give-up), then gives up as for a stuck object (TAKEN_BACK, `/join_planner/give_up`); `update_measured_drop`. Off (his default): unchanged | rig-port critic round 2, A7: an unseen puck can ride the magnet to plate 9 |
| `test/test_m1_contact_attach.py` (ours, appended) | the wait, a fresh pose away from or on the marker within it, the give-up at its end; the not-measured case no longer clears | tests first |
- **11 Oct, the drop wait as a switch (Wesley, after critic round 2 A7).** `drop_puck_wait_s` is a node parameter and
  a rig launch argument, rig default 2.0 s. Set to 0, an object not measured after the release dwell is trusted released,
  his earlier behaviour. It is for a rig where Motive cannot see the puck in the net; Wednesday's mocap check decides.

## 14 Oct 2026: rig load cuts (Wesley, after the combined stack starved our trackers in RIG-1014-t5_2; docs/analysis/rig_load_2026-10-14.md)

| file | change | why |
|---|---|---|
| `launch/partner_mission.launch.py` (ours), rig nodes only | the planner gets `visual_astar_enabled: false` | his visual-only A* (2 Hz, up to 150k expansions or 0.25 s each) only draws; the mission reference is unchanged |
| `launch/partner_mission.launch.py` (ours), rig nodes only | the planner gets `log_every_n_updates: 6` | his CSV at 5 Hz instead of 30 Hz, as the sim launch already does |
| `launch/partner_mission.launch.py` (ours), rig nodes only | the ring bridge gets `trail_seconds: 0.0` | no 20 s, 1000-point trail MarkerArray at 50 Hz; the bridge's 50 Hz outputs are unchanged |
| `launch/partner_mission.launch.py` (ours), rig nodes only | the planner gets `pickup_alignment_low_xy_m 0.045` (was 0.025), `pickup_alignment_high_xy_m 0.06` (was 0.05), `pickup_alignment_low_dwell_s 1.0` (was 0.3) | his own suggestion after RIG-1014-t5_4: the tip swung a median 3-4 cm round the puck and sat inside the 2.5 cm low tube 14-35 % of the time, so the descent stalled 9 cm up; the first attempt's lift proof failed with the face ~2 cm off the puck. The capture gate (`pickup_alignment_capture_xy_m` 0.025) and the retry settings are unchanged |
| `launch/partner_mission.launch.py` (ours), rig nodes only | after RIG-1014-t5_5 (Wesley): `pickup_alignment_low_xy_m 0.10`, `pickup_alignment_high_xy_m 0.12` (were 0.045 / 0.06 from t5_4) | with 0.045 the descent reached the bottom in 2 of 3 attempts and stalled once in HIGH; at the bottom the magnet sat 2.5-4.5 cm off the puck and the puck never rose (0.1 cm), so the catch, not the tube, is now the limit (Tejen to look at the final alignment) |

## 14 Oct 2026: floor_join and delivery_rejoin (Wesley)

| file | change | why |
|---|---|---|
| `mission_definitions.py`, `mission_types.py` | two missions from the mat, `mission_mode` `floor_join` / `delivery_rejoin` (also `floor-join` / `delivery-rejoin`): `floor_join` = his WAIT_FOR_TAKEOFF and TAKEOFF, then `join_only`'s rejoin (no pickup, no drop); `delivery_rejoin` = `pickup_delivery` without the five pickup phases, its WAIT_FOR_TAKEOFF and TAKEOFF magnet ON. `MissionDefinition.takeoff_exit` names the phase after TAKEOFF (`pickup_delivery`: APPROACH_ABOVE_PICKUP, as before) | his pickup keeps failing on the rig; `join` starts at APPROACH_ABOVE_TARGET with no takeoff, unsafe from the floor, and stays as it is |
| `online_join_planner.py` | both TAKEOFF exits (the detection height in `drone_state_callback`, "Takeoff height reached") go to `mission.takeoff_exit`; for the new modes the TAKEOFF command latches a climb height (`latch_takeoff_climb`) and the detection-height exit does not apply. `floor_join`: straight up over the mat to the approach height (plate + `approach_offset_z` + rod, at least `takeoff_height`), then APPROACH_ABOVE_TARGET. `pickup_delivery` unchanged | his Python transfer to the plate has no collision avoidance: level at the approach height, as TRANSIT_TO_REATTACH is after a drop, not a diagonal from the floor with the rod hanging |
| `online_join_planner.py` | `delivery_rejoin`: the climb ends where a loaded lift ends (tip `pickup_lift_height` above the mat), then `update_delivery_takeoff` runs LIFT_OBJECT's C1F.2b entry (object attached, object risen `lift_complete_object_clearance`, body within `loaded_lift_z_tolerance`), stationary hold, settle and C++ handoff inside TAKEOFF (`c1f2_prepare_phase` includes it); only the certified handoff enters TRANSIT_TO_DROP_POINT. Without C1F.2 the complete climb enters it directly. LIFT_OBJECT's own block is untouched | C1F.2 is on in both launch paths and TRANSIT_TO_DROP_POINT raises without the handoff; his backend plans only above `minimum_search_z_m` and with the object attached. The drop, its check (`drop_check_puck`) and the rejoin are pickup_delivery's own phase code |
| `online_join_planner.py` | `delivery_rejoin` TAKEOFF without the object reading attached (fresh) or without a fresh object pose, and `floor_join` without a fresh attachment pose: refused on the mat (`refuse_takeoff`): his `landing_disarm_command` and LANDED_DISARMED; WAIT_FOR_TAKEOFF warns beforehand. The object lost during the climb: LANDING over the mat (C1F.2 state cleared) | his MPC takes off on the same TAKEOFF message, so a planner that only stays in WAIT_FOR_TAKEOFF would fly it on that phase's reference; on the mat nothing can fall on the ring and no take-back is needed. In the climb there is no pickup to retry and the C++ transit does not plan without the object |
| `test/test_floor_join_delivery_rejoin.py` (ours, new) | the sequences, the TAKEOFF exits per mode, the magnet ON / OFF on the mat, the refusals, the loaded climb with and without C1F.2, the loss, the drop check, the launch's choices | tests first |

## 14 Oct 2026: his MPC to d00934c1 behind `mpc_profile` (Wesley: "Port his HEAD, switch"; the twin first, the rig only with Tejen's OK)

His `src/controller_mpc_payload` commits after 11a0683f: 26ff38be and e1e094ad (10 Oct), 7101b0f7 (12 Oct), 93fbf346
and d2eaf41e (13 Oct), d00934c1 (14 Oct). `mpc_profile` `oct10` (the default everywhere) is his 11a0683f controller as
flown to 14 Oct: the same weights, bounds and send path, and generated solver C identical to the cached solver flown
to 14 Oct. `oct14` is his d00934c1 controller. Neither profile is flown on this tree yet.

| file | change | why |
|---|---|---|
| `tejen_mpc/mpc_modes.py` | `MPC_PROFILES` (`oct10`, `oct14`) and `parse_mpc_profile` (anything else raises); `payload_input_rate_bounds(profile)`; `base_payload_cost_matrices(..., mpc_profile='oct10')`: `oct14` is his d00934c1 matrices (running position 200/200/80, velocity 3/3/4, swing 15/15, swing rates 4/4; terminal position 240/240/120, velocity 3/3/4, swing rates 4/4) | his 7101b0f7, 93fbf346, d2eaf41e and d00934c1 retunes, only their end values; one source for the OCP and the runtime mode matrices |
| `tejen_mpc/acados.py` | `generate_payload_ocp_controller(..., mpc_profile='oct10')`: weights and u_dot bounds from the profile (`oct14`: roll, pitch and yaw rate +-3, throttle +-0.5 as before); the profile goes into the cache identity | his d2eaf41e bounds; they are generated into the solver |
| `tejen_mpc/acados_cache.py` | `build_payload_mpc_cache_identity(..., mpc_profile='oct10')`: `mpc_profile` in the fingerprint and the manifest | the two profiles never load each other's solver; any edit to `acados.py` or `mpc_modes.py` is a new entry anyway, so both profiles need a prebuild before their first launch |
| `tejen_mpc/command_execution.py` (his, new) | `command_at_elapsed_time`, his d2eaf41e file unchanged | |
| `tejen_mpc/main.py` | parameter `mpc_profile` (default `oct10`; an unknown value logs FATAL and stops the node at startup), a startup line (INFO `oct10`, WARN `oct14`); `oct14` only: his d2eaf41e send path (the command state at the elapsed publication interval instead of the 0.1 s first-stage endpoint; the rounded sent command goes into the history), the interval timed on the node clock (his: `time.monotonic`); metadata `mpc_profile`, `command_execution`, `mpc_command_stage_duration_s` | his 30 Hz timing fix; the node clock because in the twin the plant runs on /clock (a wall interval there is a different command) and because his own P3 contract (`test_p3_sim_time_contract`, which his d00934c1 tree fails) keeps wall clocks out of the control path; on the rig the node clock is system time, the same interval to within NTP slew |
| `tejen_mpc/main.py` | his 26ff38be: `lateral_disturbance_diagnostics_enabled` (default false: the observer runs and logs, nothing reaches the MPC) and its metadata line; his IRL launch and runner wiring for it not carried | log only, default off |
| `launch/partner_mission.launch.py` (ours) | argument `mpc_profile` (default `oct10`, choices `oct10`/`oct14`) to `tejen_mpc` on the sim and the rig nodes; the rig takes it through bringup's `partner_args:="mpc_profile:=oct14"` | the switch reaches QUAD4 on both paths |
| tests | `tejen_mpc/test/test_command_execution.py` (his d2eaf41e; adapted: the bounds come from the profile, the send path is run for both profiles), `test_lateral_disturbance_passive_diagnostic.py` (his 26ff38be without its launch-wiring test), `test_mpc_profile.py` (ours: the matrices and bounds per profile, the refusal, one cache entry per profile with the real OCP builder, the startup line), `test/test_partner_mission_mpc_profile.py` (ours) | |

Not taken:

| what | why |
|---|---|
| e1e094ad: `external_reference_validate_timing` (node default false; his M1 and M2 IRL launches turn it on): a reference whose source stamp is old, more than 0.1 s ahead, duplicated, or off the 30 Hz grid is rejected, and a missing or stale source stamp sends the controller to its C.1d latched hold; with it, two CSV columns and the reject reason | a new hold that could stop QUAD4 in flight; the logging comes only with it |
| his test edits in `test_c1d_reference_fault_contract.py`, `test_c1e_logging_contract.py`, `test_p25_disarmed_reference_fast_path.py` | e1e094ad's |

## 15 Oct 2026: takeoff from the mat on the rig (plan docs/plan_after_rig_2026-10-14.md, WP8)

| file | change | why |
|---|---|---|
| `tejen_mission/online_join_planner.py` | `wait_takeoff_hold()`: WAIT_FOR_TAKEOFF's CURRENT_POSE reference holds the pose the vehicle had on the ground (updated while it is within 5 cm of the lowest pose seen), not the live pose; the accepted TAKEOFF starts from that hold | 14 Oct t6 / t6f: his MPC lifted 0.8 / 1.8 s before this planner heard TAKEOFF (another copy of a `ros2 topic pub -t 3`), and a reference that followed the measured pose let QUAD4 drift 0.4-0.5 m sideways while it climbed |
| `tejen_mission/online_join_planner.py` | `takeoff_climb_started()`: the timed TAKEOFF climb starts once the vehicle is 3 cm above the TAKEOFF start; until then the profile stays at its start (its reference is still raised to min_reference_z, so the MPC lifts). Only where min_reference_z is at least 10 cm above the start (the rig and the twin); otherwise as before | 14 Oct t6h_4: this planner took TAKEOFF 18 s before his MPC flew, so the climb ran on time from the command |
| tests | `test/test_takeoff_hold.py` (ours); `test_partner_mission_geometry.py` expects `approach_offset_y` (14 Oct `contact_dy`) | |
