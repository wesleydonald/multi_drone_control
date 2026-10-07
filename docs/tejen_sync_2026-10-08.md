# Tejen sync, 8 Oct 2026

Clone `~/tejen/drone_cage_control`, branch `SwungPayload2026_Collaborative` (remote `Mitchell-Torok/drone_cage_control`).

| item | value |
|---|---|
| local before pull | d9c69415 |
| after pull (= origin) | 611dc3d2 |
| pull | fast-forward (`--ff-only`), 13 commits (the stale tracking ref said 1) |
| since our fork point a7524bae | 14 commits, 25 Sep to 7 Oct 2026, all Te-Jen Yu |
| files changed since a7524bae | 203: 129 pure renames into `backup/`, 31 added, 43 modified (+11585 / -254) |
| unchanged in the range | his MPC source (`controller_mpc_payload`), `elrs_interface_irl.py`, `m2_irl_mocap_router.py`, `m2_irl_hardware.py`, `c1f2_handoff.py` |
| trial 3-way merge of our fork edits (scratch only) | conflicts only in `m2d_fleet_supervisor.py` (1 hunk), `m2d_sequential_supervision.py` (1), `m2d_four_drone_sequential.launch.py` (3); `online_join_planner.py`, `mission_definitions.py`, `motion_capture_publisher_irl.py` merge clean and keep our edits; every other file he changed is unmodified in our fork |

Path map: his `drone_communication` = our `tejen_mission`; `dynamic_planner_ros` = `tejen_dynamic_planner`;
`tools/standalone_dynamic_planner_cpp` = `tejen_dynamic_planner/standalone`; `controller_mpc_payload` = `tejen_mpc`.

## Changes

| file or area | what changed | class | ours today | recommendation | risk |
|---|---|---|---|---|---|
| `fake_cooperative_transport_world.py` `make_markers` (fef23390) | label marker shared the attachment `PoseStamped` and added 0.12 m before publish, so `/fake_attachment_point/pose` (his drop/rejoin plate target) was 120 mm high; fixed with `deepcopy`. His 7 Oct M1 rig run held the magnet face 106 mm above the plate at ATTACH_READY | real-world fix (bug also in sim) | bug present: `ring_bridge` inherits `timer_callback`/`make_markers`; his planner in `partner_mission.launch.py` reads that topic, so his ATTACH_READY in our M1 sim is 0.12 m higher than he intended (our launch notes "~0.18 m above the plate") | port (Wesley's word: moves the M1 hand-over point 0.12 m down) | medium: M1 sim baselines were flown with the bug; one Gazebo M1 run before any M1 claim |
| same file, `measured_stationary` mode (fef23390, 611dc3d2) | ring state from a mocap pose (ID 8), zero velocity, terminal-hold commitments | real-world (M1 ground ring), opt-in | absent | port with the row above | low (default `circle` unchanged) |
| `online_join_planner.py`: gate profiles, 5 deg takeoff swing, settle diagnostics (d9c69415, 5065bee9) | IRL-launch parameters; capture swing stays 3 deg | real-world, opt-in | absent | port (whole-file sync) | low |
| `online_join_planner.py`: landing vs permission revocation (cb5c96e8) | `M2_LANDING_STAGE`/`LANDING` no longer fault when the M2D supervisor revokes permission during fleet LAND | real-world fix, default path (M2D sim too) | absent; our M2 bench flies his M2D planner | port | low-medium: M2 bench LAND path changes; one M2 bench run |
| `online_join_planner.py`, `m2b_attachment_mission.py`, `measured_tip_alignment.py` (new) (906ee3fb, 0138b9e1) | rate-limited tip trim HIGH then LOW (fixes a repeated 5-6 cm tip offset); same-plate assignment heartbeats no longer reset the 3-attempt retry count | real-world; trim opt-in, retry count default | absent | port | low |
| `online_join_planner.py`, `m2_irl_attachment_policy.py` (new), `attachment_detection.py`, `m2_attachment_observer.py` (e89ef7ca) | `attachment_policy: measured_contact_15deg`: bounded contact search, 15 deg measured proof, quiet-miss recovery; default `legacy` | real-world, opt-in | absent | port | low |
| `online_join_planner.py`, `m1_pickup_proof.py`, `m1_pickup_recovery.py` (new) (91c0f63a, 8d6a9695, f3829d57) | M1 pickup: 0.03 m/s measured lift proof before loaded hand-off; stalled pickup retries with magnet OFF and vertical clearance first; near-surface candidates enter contact dwell | real-world (M1 rig pickup), opt-in via `m1_measured_tip_pickup.yaml` | absent; our sim M1 flies the legacy pickup | port as opt-in | low |
| `online_join_planner.py`, `m1_ground_ring.py` (new), `payload_geometry.py`, `m1_irl_ground_ring.launch.py`, `m1_ground_ring.yaml` (fef23390, 611dc3d2) | M1 onto a grounded real ring (ID 8, plate 0): magnet ON in the ring descent, ATTACH_READY auto-LAND off, ordered manual LAND (OFF, 0.25 s, rise up to 0.20 m until face gap 0.08 m, return, land) | real-world, opt-in | absent | discuss (closest to our rig M1 hand-over; his planner commands a magnet on our ring) | medium |
| `online_join_planner.py`, our edits | none from him; our transit-owner token and 1 s magnet resend | merge check | ours | keep ours (merges clean) | low |
| `m2_irl_two_drone.launch.py` (all IRL commits) | 2 or 4 vehicles from YAML, flight order, gate profiles; observer frame leaves fixed (qualified names left every 30 Sep attachment CSV empty); passive stationary seed for the first splice; controllers staggered 2 s each; still no `ELRSCommand_tejen` remap | real-world | fork-point copy, renamed | port | low (rig M2 only) |
| `m2_irl_fleet_manager.py`, `m2_irl_readiness.py`, `m2_irl_launch_config.py` | 2/4-vehicle identity; freshness classes (mocap 0.50, telemetry 0.50, ring 0.50, link stats 0.75, status 1.25 s); plate-assignment markers; ring geometry from YAML; validated gate profiles | real-world | fork-point copy | port | low |
| `m2_irl_two_drone.yaml` | tether 0.475 to 0.5575 m (0.575 body-to-marker + 0.0225 face - 0.040 anchor); contact face -0.0225 m; ring annulus 0.385/0.510 m (plate pitch diameter 0.4475); gate profiles; still drone_0 = QUAD2, drone_1 = QUAD4 | real-world calibration | fork-point copy | port | low |
| `m2_irl_four_drone.yaml` (new) | drone_N = QUAD(N+1), bodies 11-14, magnets 21-24, ring 8, FTDI serials, magnet_channel 6, flight order drone_2, drone_0, drone_1, drone_3 | real-world | absent; same map as ours | port | low |
| `motion_capture_publisher_irl.py` | M1 default IDs quad 7 to 12, magnet 8 to 22; optional ring publisher (`ring_pose_topic`); rejects duplicate IDs; still `return None` on a bad packet | real-world | has our receive-loop fix (f8c3c66) | port, keep our fix (merges clean) | low |
| `irl_commissioning.yaml` | `magnet_drop_below_quad` 0.53 to 0.575; marker-to-face -0.030 to -0.0225 (both nodes); his MPC `xy_bias_mode` lateral_disturbance to legacy_integral | real-world calibration (M1, QUAD2) | fork-point copy; our partner sim MPC already legacy_integral | port | low |
| `m1_measured_tip_pickup.yaml` (new), `m1_irl_virtual_ring.launch.py`, `c1f_irl_m1_virtual_ring_backend.launch.py` | M1 overlays: `pickup_alignment_mode` arg (default legacy), ID log 12/22, `ring_commitment_topic` arg | real-world, opt-in | absent / fork-point copies | port | low |
| `transfer_backend_node.cpp` (906ee3fb) | `cooperative_shared_start_skew_s` (max 0.05 s, peers at most 0.05 m/s) accepts a peer commitment that starts just after the latest peer state; default 0 | real-world (30 Sep: 26 cooperative-input disablements in one run) | fork-point copy; our notes item 11 left "why the backend disables mid-transit" open | port | low (default off); possible cause of our M2 stall, untested |
| standalone `receding_horizon_planner.cpp`, `world_snapshot.cpp` + 2 C++ tests (82fbff07) | recheck runs from the post-snapshot time with explicit coverage errors; identical cooperative updates no longer bump the world version | real-world (2 Oct rig run), default in every mode | fork-point copy in `tejen_dynamic_planner/standalone` | port | medium: default planner behaviour in our M1/M2 sim; rebuild, one M1 and one M2 bench run |
| `m2d_fleet_supervisor.py`, `m2d_sequential_supervision.py` | progressive policy: operator wait instead of abort on proven loss; `fresh` flag on attachment evidence; opt-in | real-world + sim; conflicts with our fixes | ours adds `simultaneous` mode and `/m2d/transit_owner` | port by hand | conflict: 1 hunk each |
| `mission_definitions.py` | phase descriptions; `attachment_policy` argument | refactor/cosmetic | ours sets rejoin magnet ON (4 lines) | port (merges clean) | low |
| `m2d_four_drone_sequential.launch.py`, `tools/sim_test/run_m2d_sequential_attachment.sh` | `attachment_policy` launch arg (default legacy), OpaqueFunction restructure, `M2D_SKIP_CLEANUP` | sim-only; conflicts with our fixes | ours edited the launch | skip or port by hand | conflict: 3 hunks; low value |
| `tools/sim_test/run_m1_irl_virtual_ring.sh` | IDs 12/22; pickup alignment mode arg (a rig runner despite the folder) | real-world | fork-point copy | port | low |
| `tools/irl_test/`: `99-elrs-m2.rules`, `run_m2_irl_four_drone.sh`, `latest_m2_irl_log.sh`, `run_m1_ground_ring.sh`, `check_m1_ground_ring.py` | udev template now has all four adapters; four-drone and ground-ring runners; props-off ring check | real-world | our copy of the rules is the 2-adapter one; installed `/etc/udev/rules.d/99-elrs-quad.rules` has the same four serials and names | port | low (our notes item 5 resolved) |
| `drone_visualisation` `m2_fleet_panel.cpp/.hpp` | voltage column (cached ELRS values), RETRY ACTIVE / LAND ACTIVE buttons | real-world (operator UI) | we do not have his panel (ours: `arm_panel`) | skip | none |
| `quad_payload_integral_dynamics_ocp.json` (repo root) | regenerated acados JSON: xy-integral weights 0 to 15 (stage) and 30 (terminal) | refactor/cosmetic (generated) | `tejen_mpc` builds into an acados cache | skip | none |
| tests: 11 new, 6 modified Python; 2 modified C++ | travel with the code; his notes list 3 to 5 stale lab-YAML assertions | test | absent | port with code | low |
| docs: `docs/thesis/*`, `docs/exec-plans/*`, `REPO_INDEX.md`, 4 `commandsFor*.txt`, `experiment-history.md.orig` (1924-line stray copy) | evidence, safety contracts, operator sheets | docs | n/a | skip (read for evidence) | none |
| 129 renames into `backup/`, `.code-workspace` | housekeeping | refactor/cosmetic | n/a | skip | none |
| thrust map (his MPC) | no change: linear kT with `full_model_kt_ukf` and feedback; his 2 Oct logs show learned kT about 22.2-22.3 | none | ours: affine map (offset 0.185, gain 35.2, voltage term), `kt_trim` off | skip; offer him our map | none |
| arming / ELRS command path | no change; his `Telemetry` is republished at 10 Hz from cached values, so his 0.50 s telemetry gate cannot see QUAD1's 2-3 s flight-mode gaps, and nothing checks the FC armed state | none | ours: per-attempt after-edge confirmation (`arm_switch.py`), TAKEOFF refused unless every FC reports armed (`_wait_fc_armed`) | skip (ours is stricter); tell Tejen, his four-drone config flies QUAD1 | his side only |
| mocap dropout (his docs, 30 Sep) | shared mocap gap of about 0.35 s; quad fell from 0.73 to 0.07 m; consistent with his 0.25 s armed-pose watchdog; left open by him | real-world finding, no code | our rig `pose_timeout_s` 0.25 with fleet-wide disarm: the same gap grounds our fleet | discuss (threshold needs Wesley's word) | high if it recurs |
| vertical oscillation (his docs, 2 Oct four-drone run) | QUAD3 Z 51 mm peak-to-peak at 0.88 Hz in free hover; QUAD1 grew to 228 mm in attached hold (1.64 Hz); cause unresolved (airframe vs third controller's load) | real-world finding, no code | same airframes | discuss | n/a |

## Port plan (after tomorrow's rig session, branch `tejen-sync`, only with Wesley's word)

1. Marker aliasing fix in `fake_cooperative_transport_world.py` with his test; then one Gazebo M1 partner run, because his hand-over point moves 0.12 m down.
2. Three-way merge of `tejen_mission` from a7524bae to 611dc3d2 (names mapped); resolve the 5 conflict hunks in the three M2D files keeping our `simultaneous` mode and transit token; keep our receive-loop fix, magnet resend and rejoin magnet ON; add his new modules, configs and tests; pytest `tejen_mission`.
3. C++ planner fixes (`world_snapshot.cpp`, `receding_horizon_planner.cpp`, `transfer_backend_node.cpp` + tests) into `tejen_dynamic_planner` and `standalone/`; rebuild; one M2 bench run and one M1 run; check whether the M2 join stall (notes item 11) recurs.
4. Rig M2: `m2_irl_four_drone.yaml`, the generalised launch, `run_m2_irl_four_drone.sh`, udev template; then our own P14 (his MPC to `ELRSCommand_tejen`) and P12 (one mocap owner) in our copy.
5. M1 rig overlays (measured-tip pickup, proof, recovery, ground ring) as opt-in only; whether combined M1 on the rig uses them is Wesley's call.

Skip: his fleet panel, the OCP JSON, docs, `backup/` moves. Tell Tejen: our receive-loop fix and padding fix are still not in his tree; QUAD1's sparse flight-mode telemetry and the missing FC arm check.

## Interface checks

| interface | status | detail |
|---|---|---|
| drone IDs | CHANGED (now matches ours in the four-drone YAML) | four-drone YAML: drone_N = QUAD(N+1), bodies 11-14, magnets 21-24, ring 8, serials as our udev. His launch default is still the two-drone YAML (drone_0 = QUAD2, drone_1 = QUAD4) and M1 flies QUAD2 (body 12, magnet 22) as drone_0. Combined runs: use the four-drone YAML |
| mocap owner (UDP 1511) | CHANGED, still unresolved | M1 publisher now also reads ring ID 8 (opt-in) on its own 1511 socket; M2 router unchanged (binds 0.0.0.0:1511, no SO_REUSEADDR). Still one binder per stack; our P12 open. His 2 Oct runs used bodies 11-13, magnets 21-23 and ring 8, which answers part of our Motive question |
| magnet ownership | CHANGED, still unagreed | his planner now issues more magnet commands (M1: OFF before a retry and at LAND near the ring, ON in the ground-ring descent; M2: kept through `M2_FAULT`, released at operator LAND), still through his `elrs_interface_irl` (`/drone_i/magnet/ELRSCommand`, channel 6 on all four = our `magnet_channel` 6). Who owns the channel in a combined run (notes item 7) is open |
| `ELRSCommand_tejen` remap | OK (unchanged) | his MPC still publishes `/drone_i/ELRSCommand` straight into his radio node (remaps only arming/command names); the remap exists only in our sim `partner_mission.launch.py`; the rig remap (P14) is still ours to add |
| hand-over topics | OK names, CHANGED semantics | `/join_planner/handoff_ready`, `/join_planner/reference`, `/join_planner/phase`, magnet/command, object_attached, `/m2c/*`, `/m2d/*` keep their names; new `/m2c/assignment_markers`, `<ns>/attachment/proof_motion_complete`, `<ns>/join_planner/retry`, `<ns>/join_planner/land_now`, `/irl/ring/*`. After the aliasing fix his ATTACH_READY sits 0.12 m lower on the virtual-ring path (ours in sim) |
| attach geometry (extra) | CHANGED | his tether 0.5575 m anchor-to-face, anchor -0.04, plate pitch radius 0.22375 m; ours `cable_len` 0.55 (measured), pivot -0.04, `attach_radius` 0.225. Differences 7.5 mm and 1.25 mm |
