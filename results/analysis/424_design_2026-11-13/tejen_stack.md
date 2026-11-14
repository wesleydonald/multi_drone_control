# 4 -> 2 -> 4: two instances of Te-Jen's stack side by side

Design study, 13 Nov 2026 (read-only; tree as on disk, `week8` + working copy). Scope: his nodes and
`partner_mission.launch.py`; our mux, gate, proof and planner appear only where they meet his stack (the "our stack"
study owns them).

## Verdict

**Feasible, with launch plumbing only. Two instances need no change to how his planner, C++ backend or MPC behave.**
Every topic his nodes use is a parameter or a relative name, except two hard-coded topics in his planner and one in
our ring_bridge. All three can be remapped in the launch. His own M2 launches already run one namespaced
planner + backend + MPC per vehicle (`m2_irl_two_drone.launch.py:180-370`, `m2d_four_drone_sequential.launch.py:488-493`;
four at once in our twin, T0015). Our `partner_mission.launch.py` is single-vehicle: it puts the planner, backend and
ring bridge in the root namespace on fixed global names. It also hard-codes drone 3, `/partner/release`, QUAD4's tip and
three carriers.

**Two problems to plan around:**
1. **The two returning quads cannot see each other.** Neither the Python transfer that flies the re-attach today
   (`attach_cpp_transit:=false` on every rig detach/re-attach line) nor his C++ planner avoids the other returning quad.
   Making them see each other means changing his planner, which needs his OK. His C++ planner also cannot certify
   paths around a moving peer, so two transits at the same time get serialised or stall (T0014b). For 18-19 Nov, the
   separation has to come from the geometry: opposite plates, and each quad stays on its own side of the ring.
2. **Leaving any default in place breaks things.** Two traps:
   - The MPC's default reference topic is the absolute `/join_planner/reference`. If it is left unset, both MPCs follow
     both planners.
   - His YAML files are keyed by node name (`online_join_planner:`, `dynamic_planner_transfer_backend:`). rclpy and
     rclcpp match those keys against the node's full name (`/online_join_planner`), so a namespaced node silently gets
     none of `c1f2b_cpp_authority.yaml`, `m1_measured_tip_*.yaml`, `c1f6_moving_rendezvous.yaml` or our
     `partner_backend_overrides_rig.yaml`. It would fly on his code defaults. We checked the matching in the source
     (`~/ros2_humble/src/ros2/rclpy/rclpy/src/rclpy/node.cpp:274-290` and
     `rclcpp/src/rclcpp/parameter_map.cpp:54-63`).

   His M2 launch loads every block as a dict for exactly this reason. A launch-dump test must guard both traps.

**Recommended shape:**
- Instance A (QUAD4 = drone_3, plate 9) stays byte-identical to today, so every single-QUAD4 line, tool and bag list
  keeps working.
- Instance B (the carrier, e.g. drone_1 = QUAD2 on plate 3 in the 0/90/180 cw layout) runs entirely under its own
  namespace (`/tejen_d1`), with every topic written out and its backend built in our launch from dicts.

## His id mapping ("vehicle_id=drone_0" while flying our drone_3)

- `vehicle_id` is his name for "the ego vehicle" and only a label in M1. Both his planner and his backend default it
  to `drone_0` (`online_join_planner.py:367`, `transfer_backend_node.cpp:593`, `c1f6_moving_rendezvous.yaml`
  `vehicle_id: drone_0`), and so do his single-vehicle M1 launches. `partner_mission` types it again
  (`partner_mission.launch.py:713, 1138`), and `assigned_attachment_id: attachment_0` is a label too.
- In M1 it is used only for:
  - the planner CSV name (`online_join_planner.py:11601`);
  - `CommittedTrajectory.vehicle_id` (`:9912`, published only in M2);
  - the reference's `joint_names` (`reference_message.cpp:97`);
  - the M2 transit token compare (`:3769`).

  Nothing in M1 checks it.
- His M2 fleet identity is the same as ours: drone_N = QUAD(N+1), radios `/dev/QUADn`, magnet markers 21-24, magnet
  channel 6 (`config/m2_irl_four_drone.yaml:1,21-51`). His `drone_0` label in M1 is therefore QUAD1 in his own M2 naming,
  not QUAD4. Give the two instances `vehicle_id` drone_3 and drone_1. This is behaviour-neutral, and it separates their
  CSV files, which otherwise collide.
- The `/fake_obstacles/drone_i/*` topics are not our drones either. They are the i-th entry of
  `carrier_azimuths_deg` (`ring_bridge.py:127-139`; count = `len(azimuths)`,
  `fake_cooperative_transport_world.py:298`).
- His M1 was commissioned on QUAD2: quad body 12, magnet marker 22 (`irl_commissioning.yaml`
  `motion_capture_publisher_irl`). A pickup line also flew QUAD1 (`m1_irl_pickup.launch.py:126-129`). So his MPC has
  already flown two of our carriers.

## 1. Inventory: what is global or hard-codes drone 3 / QUAD4 / plate 9

**Key**
- **ns**: a ROS namespace fixes it (relative name).
- **param**: already a parameter; set it per instance in the launch.
- **remap**: a launch remap of an absolute name. ROS 2 remap rules match the full name, so absolute names remap too.
- **code**: needs an edit.

All of these are fixable without touching his planner's behaviour.

### partner_mission.launch.py (ours)

| line | what | fix |
|---|---|---|
| 63-66 | `NS='tejen'`, `DRONE=3`, `DRONE_MODEL='x3_drone3'` (sim), `OBJECT_POSE` (sim) | launch args `instance` / `drone_id`; A keeps the defaults |
| 502-505 | `CONTACT_TOPICS` (`/magnet/contact`, `/magnet/object_attached`, `/magnet/attach_proof`, `/join_planner/give_up`, `/drone_{DRONE}/take_back`, `/drone_{DRONE}/mux_state`), `RELEASE_GATE='/partner/release'` | per instance (inputs come from our side per drone) |
| 984 | `plate`, `carrier_azimuths_deg` -> his plate index and azimuths | already typed per instance (`plate:=3` -> his 9); carriers = the two that stay, e.g. `[0.0,180.0]` |
| 987-988 | `state=/drone_3/motion_capture_state`; tip `/tejen/magnet_tip_pose`, ball `/tejen/pickup_object/pose`, `/tejen/object_attached` | derive from `drone_id` / `instance` |
| 1053-1092 | ring_bridge named `fake_cooperative_transport_world`, root namespace; all its output topics at his absolute defaults (`/fake_payload/*`, `/fake_attachment_point/*`, `/fake_obstacles/*`, `/fake_world/markers`, `/fake_payload/pose_array`); `fake_drone_count: 3` (cosmetic, overridden by the azimuth list) | B: `namespace` + every topic parameter written out (`fake_cooperative_transport_world.py:170-182`, `ring_pose_array_topic`); its attach point is plate 3's via `bridge_attach_xy_m` / `bridge_attach_z_m` (contact_target_xy/z) |
| 1095-1101 | partner_rig_state: `body_topic=state`, `tip_topic='/magnet_tip_pose'` (QUAD4's marker 24 only) | B: `ns=/tejen_d1`, body `/drone_1/motion_capture_state`, tip = drone 1's marker (see partner_rig_state below) |
| 1105-1122 | his ball manager (object only) | B: leave out (no object in 4->2->4) or namespace it |
| 1125-1138 | planner `online_join_planner` in root; YAML files `visual_astar_visual_only`, `c1f2b_cpp_authority`, `m1_measured_tip_pickup/_tracking` passed as files keyed `online_join_planner:`; `vehicle_id: drone_0` | B: `namespace`; the four files as dicts (`_irl_block(share, 'online_join_planner', <file>)`); `vehicle_id: drone_1` |
| 1142-1154 | planner topics: state, `/tejen/drone_command`, ball, `/tejen/magnet/command`, tip, `/fake_payload/*`, `/fake_attachment_point/*`, `/join_planner/land_now` | B: its own names |
| (unset) | planner defaults the launch never types, absolute: `reference_topic` `/join_planner/reference`, `phase_topic`, `state_topic`, `marker_topic`, `handoff_ready_topic`, `obstacle_diagnostics_topic`, `operator_advance_override_topic`, the seven c1f1/c1f2 `/dynamic_planner/*` topics (and `c1f2b_cpp_authority.yaml` sets the same seven absolute), the three `c1f2_recovery_*`, `proof_lift_complete_topic`, `log_directory` | B: every one typed under `/tejen_d1` **after** the c1f2b dict (later wins) |
| 1206 | `ring_pose_topic: /payload/motion_capture_state` | shared (the same ring): keep |
| 1216-1220 | `contact_topic /magnet/contact`, `ring_attached_topic /magnet/object_attached`, `give_up_topic /join_planner/give_up`, `take_back_topic /drone_3/take_back`, `mux_state_topic /drone_3/mux_state` | per drone; our side must publish drone 1's |
| 1273-1286 | backend through his `c1f6_moving_rendezvous_backend.launch.py`: node in root; `cooperative_preview_count '3'`; `extra_params_file` keyed `dynamic_planner_transfer_backend:` | B: build the backend Node here, from dicts, as `m2_irl_two_drone.launch.py:325-370` does (his launch's inline dict comes last, so files cannot override its cooperative topics or `csv_path`) |
| 375-397 | `_backend_overrides`: `cooperative_attached_plate_topics` hard-coded `/fake_obstacles/drone_{i}/attached_plate for i in range(3)`, `cooperative_ring_pose_topic /fake_payload/pose_array` | per instance; the count from the carrier list |
| 1288-1310 | MPC: `namespace=NS` ok; remaps `motion_capture_state`, `ELRSCommand -> /drone_3/ELRSCommand_tejen`, `telemetry -> /drone_3/telemetry`; **not set**: `external_reference_topic` (default `/join_planner/reference`), `payload_mpc_mode_topic` (default `/join_planner/mpc_mode`, always subscribed, `main.py:513`), `logging_instance` | B: remaps to `/drone_1/*`; set the three |
| 1336-1343, 1403-1405 | bag (`record:=true`) topic list and `bag_path` / `result_path` defaults | per instance (the rig records with `tools/rig_flight.sh`, below) |
| sim path 612-936 | twin only: pendulum from `/model/x3_drone3/pose` link indices (657-668); gz object bridges and the `gz topic -t /pickup/detach` call (612-614, 911-924); supervisor (877-908) | see section 5 |

### His nodes

| file:line | what | fix |
|---|---|---|
| `online_join_planner.py:408-480, 496-517, 603-630` | every topic is a parameter; absolute defaults (`/motion_capture_state`, `/fake_*`, `/join_planner/*`, `/magnet/*`, `/dynamic_planner/*`) | param |
| `online_join_planner.py:2058-2072` | contact-attach topics; defaults `/drone_3/take_back`, `/drone_3/mux_state` | param |
| `online_join_planner.py:2767` | `'/join_planner/force_drop'` hard-coded | remap (or code: make it a param, S, no behaviour change) |
| `online_join_planner.py:2826` | `'/dynamic_planner/static_target'` hard-coded (only `return_after_drop:=cpp`) | remap; unused in 4->2->4 |
| `online_join_planner.py:1284-1294` | passive moving-obstacle tube (RViz only, "does not ... modify ... the controller reference"), subscribes `/fake_moving_obstacle/state` by default | harmless (nothing publishes it); remap for tidiness |
| `online_join_planner.py:735-736, 806, 822` | `/experiments/*`, `/join_planner/arm_permission`, `/join_planner/mpc_mode` | only with preloaded_navigation / m2b / m1_gentle_takeoff: off in 4->2->4 |
| `online_join_planner.py:996-1000` | `join_planner/operator_events`, `.../retry`, ... | relative: ns |
| `online_join_planner.py:11591-11604` | CSV `logs/join_planner/join_planner_clean_<second>_<vehicle_id>.csv`, relative to the cwd | per-instance `vehicle_id` (two `drone_0` in one second share one file) |
| `transfer_backend_node.cpp:558-593` | node name `dynamic_planner_transfer_backend`; every topic a parameter with absolute defaults (`/motion_capture_state`, `/dynamic_planner/*`, `/magnet/object_attached`); `vehicle_id drone_0` | param |
| `transfer_backend_node.cpp:786-793, 824-827, 851` | the peer lists, attached-plate topics, ring pose (default `/model/payload_model/pose`), moving basket `/fake_payload/committed_trajectory` | param |
| `c1f6_moving_rendezvous_backend.launch.py:33-36, 39-62` | peer topics hard-coded to `/fake_obstacles/drone_{i}/...`; node in root; YAML keyed by node name; run directory `logs/tejen_dynamic_planner/c1f6_rendezvous_<second>[_n]` (unique, but which instance is which shows only in its LogInfo line) | do not include it for B; build the Node in partner_mission with `csv_path` per instance |
| `tejen_mpc/main.py:62, 79, 101, 236, 441, 902-906` | node `controller` (renamed by the launch); absolute defaults `/join_planner/arm_permission` (opt-in), `/join_planner/mpc_mode`, `/join_planner/reference`; log `cwd/logs/tejen_mpc[/<logging_instance>]/external_reference_<second>/log.csv` | param (`logging_instance` per instance: two MPCs that start in the same second open the same `log.csv` with `'w'`) |
| `tejen_utility_objects/callback_manager.py:20-32` | `ELRSCommand`, `motion_capture_state`, `telemetry`, `drone_command`, `drone_arming_state_feedback`, `pendulum_swing_state`, service `drone_arming_service` | relative: ns + remaps |
| `tejen_utility_objects/visualization.py:44-53, 119` | RViz topics relative; TF child frame `<namespace>/drone_mocap` | ns (already designed for one controller per namespace) |
| `tejen_mpc/acados_cache.py:45-74, 132-142` | shared solver cache `~/.cache/multi_drone_control/tejen_payload_mpc` | fine to share: the fingerprint has no vehicle values (kT, cable and rates are runtime parameters, `main.py:556-561`) and compilation is behind `flock` |
| `magnet_attachment_manager_irl.py:42-55, 106` | topics are params (defaults `/magnet/*`, `/magnet_tip_pose`) | param |
| `magnet_attachment_manager_irl.py:161` | `/join_planner/phase` hard-coded | only with `preloaded_takeoff_attachment_override` (off) |
| `simulation_test_supervisor.py:391-409` | absolute `/join_planner/phase`, `/join_planner/handoff_ready`, `/motion_capture_state`, `/pendulum_swing_state`, `/payload_world_state`, `/dynamic_planner/transfer_status`, `/join_planner/land_now`, `/join_planner/give_up` | remap (twin only; the rig refuses his supervisor) |

### Our nodes inside his packages

| file:line | what | fix |
|---|---|---|
| `ring_bridge.py:60-62` | reads `/payload/trajectory_state`, `/payload/motion_capture_state` | shared by both instances: keep |
| `ring_bridge.py:68` | `f'/fake_obstacles/drone_{i}/attached_plate'` hard-coded | code S (a prefix parameter) or remap |
| `partner_rig_state.py:71-82` | `ns`, `body_topic`, `tip_topic` params; the tip must be a `PoseStamped` (`/magnet_tip_pose`, QUAD4's marker from `mocap_magnet_tip_body_id`) | code S: accept drone 1's marker as the `MotionCaptureState` our mocap already publishes for bodies 21-24 (`/drone_<d>/magnet_tip_state`, `mocap_extra_bodies`), or have our mocap publish a second PoseStamped |

**No global parameters, services or TF frames collide** once the nodes are namespaced. His mission nodes broadcast no TF. The
MPC's TF child is namespace-prefixed and its arming service is relative.

### Where his topics meet ours (owned by the "our stack" study; listed so both halves match)

| our file:line | his topic or value it hard-codes |
|---|---|
| `graph_attach.py:47-48, 59` | `ATTACH_DRONE_ID = 3`, `x3_drone3`, `COIL_TOPIC /drone_3/magnet_coil` |
| `graph_attach.py:179-206` | one partner include, with plate 9's `weld_point_z` and attach offsets, QUAD4's `proof_taut_len`, `/partner/release` |
| `graph_attach.py:775-797, 991-1003` | `elrs_mux_3`; `magnet_gate_3` reading `/tejen/magnet/command`, `/join_planner/phase`, `/tejen/object_attached` |
| `elrs_mux.py:140-156` | per `drone_id`, but `/magnet/object_attached` global |
| `dissipative_node.py:716, 920-922` | publishes `/partner/release`; reads `/join_planner/phase`, `/join_planner/give_up`, `/tejen/object_attached` |
| `magnet_attachment_manager.py:204-215` | `/magnet/contact`, `/magnet/attach_proof`, `/join_planner/proof_lift_complete`, `/join_planner/phase` (params) |
| `tools/console.py:49-58, 460-464`; `tools/his_console.py:24-26` | `/tejen/drone_command`, `/join_planner/land_now`, `/join_planner/force_drop`, `HIS_NODES` guard, `/tejen/drone_arming_state_feedback`, `/join_planner/phase` |
| `tools/rig_flight.sh:86-104` | bag list: instance A's topics only |
| `tools/experiment/runner_node.py:173-245` | twin runner: his topics, single instance |

## 2. Does his planner see the other returning quad?

**No, in either route.**

- **The Python transfer (the direct route).** This is what flies every rig detach/re-attach today
  (`attach_cpp_transit:=false`, e.g. RIG-1111-nrtfc_2). It has no avoidance at all.
  - `APPROACH_ABOVE_TARGET` is a smooth transfer from the measured state to the body target above the plate
    (`online_join_planner.py:9418-9426`, `mission_definitions.py:257-330`), optionally via a line-up point.
  - The "safety planner" is diagnostics-only (startup log `vectorised_whole_body_diagnostics_only`).
  - The moving-obstacle tube is display-only (`:1280-1283`).
  - `static_obstacle_count: 0`.
- **The C++ planner (`attach_cpp_transit:=true`, or any C++ leg).** Its peers are exactly
  `cooperative_drone_state_topics` / `cooperative_committed_trajectory_topics`. In partner_mission these are the
  ring_bridge's fake carriers: fixed points in the ring frame at the cable elevation ("three_attached",
  `ring_bridge.py:127-142`). The other returning quad is not on the list, so each instance flies blind to the other.
- **Adding the other quad as a peer is not a parameter change:**
  - In M1 his planner publishes a commitment only when it is grounded and still (`publish_committed_trajectory`,
    `:12957-12962`, `:9886-9922`). The moving commitment exists only with `m2d_enabled` (`:9924-9958`), and that also
    switches on M2 mission gating (`:771-772, 3848, 4064, 4574, 4975`). The backend refuses to plan while any listed
    peer lacks a fresh, valid commitment (`transfer_backend_node.cpp:1927-1971, 2020-2029`).
  - Even with a moving commitment (his M2 reference window, about 2 s), his checker needs each peer's commitment to
    cover the whole check interval, unless the peer ends in a stopped hold (`trajectory_geometry.cpp:124-127`;
    `receding_horizon_planner.cpp:827-838`). It rejects candidates otherwise.
  - On 26 Sep this stalled his own M2 joins when two transits ran at once (T0012-T0014, root cause T0014b). His M2D is
    sequential by design. Our fix was a transit token, one C++ transit at a time (`m2d_transit_owner_topic`, read only
    in the M2 takeoff hover, `:3765-3775`).
  - So mutual C++ avoidance for two simultaneous transits means: a new M1 commitment publisher, the peer in each
    backend, and serialised transits. That is a change to his planner's behaviour and needs Te-Jen's OK. Size L.
    Not before 18-19 Nov.
- **ring_bridge carriers for 4->2->4.** List only the two carriers that stay (e.g. `[0.0,180.0]`). The backend's
  `cooperative_preview_count` and `_backend_overrides`' `range(3)` must follow that list. Today both are hard-coded to
  3, so with two carriers the backend would wait for ever on `/fake_obstacles/drone_2/*`
  (`WAITING_FOR_VALID_COOPERATIVE_SHARED_TRAJECTORIES`). That does not matter for the Python route, but it would block
  any C++ leg. Do not list the departed carrier either: instance B's own plate would then sit inside a phantom obstacle.
- **Geometry when both re-attach at once on plates 9 (+y) and 3 (-y), carriers on 0 and 6 (+-x), M2A ring
  r 0.225 m:**
  - **Release.** Our planner releases each quad 0.5 m out and 0.3 m up along its own radial line
    (`partner_release_out_m` / `_up_m`, `dissipative_node.py:693-695`). His TAKEOFF climbs, then the transfer flies
    in along the same line to above its plate. Each quad stays in its own half-plane (y > 0 or y < 0). The carriers sit
    on the dividing line, 0.725 m from the centre and about 0.46 m below the approach height (body ~1.33 m over the
    plate: tip 0.30 m + ~1.03 m taut cable; carrier bodies ~0.87 m over the plate at 60 deg).
  - **Closest approach.** The two quads are closest with both over their plates: bodies about 0.45 m apart centre to
    centre, about 0.24 m between 0.21 m airframes, and tips 0.45 m apart. The descent bands are 5 cm, and the tip swings
    a median 3-4 cm.
  - **When the quads would cross.** Only if a quad is released on the wrong side, or starts its climb before its own
    release. This has happened once: on RIG-1105-nr1r a TAKEOFF pressed during the 20 s route flew QUAD4 1.3 m across
    the ring, which is why `takeoff_gate_topic` exists. With two instances, each gate must be its own drone's release.
    If both are left on `/partner/release`, the first release starts both quads.
  - `line_up:=gap` with carriers 0/6 lines each quad up on its own plate's line (gap bisectors 3 and 9). It is
    available but not needed.

## 3. The attach target: can instance B target plate 3?

**Yes, by launch values alone.**

1. `plate:=3` becomes his plate 9 for the ring_bridge (`_his_ring`, `tejen_plate`, `partner_mission.launch.py:512-515`).
2. The ring_bridge publishes the target point:
   - xy is `bridge_attach_xy_m` (our weld point; `contact_target_xy` = `attach_x_offset,attach_y_offset`), else his
     nominal plate;
   - z is `bridge_attach_z_m` (`contact_target_z` = `weld_point_z`);
   - both are rotated by the full ring attitude (`ring_bridge.py:179-201`).
3. It publishes that point on `attachment_pose_topic` / `attachment_twist_topic`, which the planner follows in
   APPROACH, MATCH, DESCEND, LIFT_PROOF and the bottom hold.

Nothing else in his planner assumes plate 9:
- the line-up uses the plate's radial line (`:180-208`);
- the bottom hold works in the ring frame from `ring_pose_topic` (`:3094-3110`);
- his measured-tip trim learns per instance.

**Instance B needs plate 3's own numbers:** a `weld_point_z` calibration with the face on plate 3, plate 3's weld point
xy (nominal (0, -0.225); plate 9's measured value is (0, +0.2293)), and drone 1's taut body-to-marker length
(`magnet_drop_below_quad`, refused untyped, `partner_mission.launch.py:969-972`). Today `graph_attach.py:186-198` passes
plate 9's and QUAD4's values: one set per include.

## 4. His MPC on a carrier

- **Per drone: yes.** One MPC process per radio stream: namespace + remaps (`ELRSCommand -> /drone_1/ELRSCommand_tejen`,
  `motion_capture_state -> /drone_1/motion_capture_state`, `telemetry -> /drone_1/telemetry`). Set
  `external_reference_topic` and `payload_mpc_mode_topic` to B's planner, and `logging_instance: drone_1`.
- **One stack per radio** (6 Nov, RIG-1106-q4f): each radio needs exactly one writer. On the rig that is the mux, so
  drone 1 needs its own `elrs_mux_1`, with our tracker_1 remapped to `/drone_1/ELRSCommand_diss`. This is our side;
  graph_detach's `partner_m2` already does it per drone. The mux needs `start_ours`, `zero_partner_aux` (his MPC sets
  aux 5-10 to +1; magnet channel 6) and `partner_hold_until_planner`. Never `--once` for his ARM/TAKEOFF/DISARM.
- **Thrust.** His MPC does not use our affine map. It flies its own kT (`thrust_ratio`, 22 for QUAD4, typed as
  `mpc_thrust_ratio`, refused untyped on the rig), with his kT UKF and feedback (+-25 %, `irl_commissioning.yaml`
  `tejen_mpc`). 22 is also his QUAD2 commissioning value. `mpc_battery_comp:=0.022` is the rig's identified slope for
  every quad. A carrier that has lifted the ring has a more tired pack, which is what the battery term and his UKF are
  for.
- **Mass.** `object_vehicle_mass_kg` is used only in the object gain schedule (off on the rig, `object_mass_kg 0`). All
  quads are 0.55 kg.
- **Cable.** `mpc_cable_length` is typed per quad (the rig lines type 1.0). `magnet_drop_below_quad` is drone 1's taut
  body-to-marker length from its own calibration. Both are runtime parameters, so the cached solver serves both
  instances.
- **Rates.** `rates_centre_deg` / `rates_max_deg` default to 100/100 with expo 0. That matches Betaflight rate profile 3
  as read on QUAD3 (`docs/rig/archive/betaflight_2026-10-22.md`; QUAD1/2/4 were not read, only assumed the same).
  Before his MPC flies a carrier, read that carrier's rates (read-only CLI). Any change needs the supervisor.
- **Radio link.** QUAD1-3 run ELRS mode 8 (333 Hz Full), QUAD4 mode 12 (F500) (same note). His MPC sends at 30 Hz, so
  this should be minor, but every rig tuning of his so far was on QUAD4.
- **Magnet marker.** Drone 1's marker is Motive body 22 (his M2 identity). Streaming bodies for 4->2->4 without the
  object: 8, 11-14, 22, 24 = 7, under the 8-body cap.
- **Start-up.** Like QUAD4, the carrier is armed idle by his ARM, held by its mux until our release, and learns kT in
  the air (it is armed on the floor, more than 0.20 m below its release). No change.

## 5. A twin test with two instances

His stack flies in the twin through partner_mission's sim path, e.g. `configs/experiments/rig_nrtf_z12_twin.yaml`
(R4243). That path is built for QUAD4 only:
- `pendulum_state_publisher` reads `/model/x3_drone3/pose` by link index (`partner_mission.launch.py:657-668`);
- the gz object weld is QUAD4's (`/pickup/*`);
- his supervisor hard-codes global topics (remap).

**What two instances need in the twin:**
1. **A rig-like node set for instance B.** Use partner_rig_state on drone 1's body
   (`/drone_1/motion_capture_state`) and its tether tip, instead of the x3 pendulum publisher. The emulator already
   publishes `/drone_<i>/magnet_tip_state` (`payload_mocap_emulator.py:16, 136`, `tether_tips`). There is no object, and
   the supervisor is remapped into B's namespace (or ARM/TAKEOFF come from the runner). Size M, ours.
2. **A world and config where a carrier can be released to his stack and re-weld on its own plate.** The lift_system
   worlds already have `/drone_i/attach` on each tether's DetachableJoint
   (`three_rigid_ground_rig_attach_welded_pickup_L100.sdf:386-415`). It also needs the square layout (carriers 0/3/6 +
   QUAD4 9, as the `sq_*` worlds), `mux_1` / `magnet_gate_1` / drone 1's contact and proof / per-drone release in our
   graph, and per-drone DETACH and WAIT_REWELD in the runner (`runner_node.py` hard-codes his topics). This is the
   twin-plan and our-stack studies' item.
3. **The twin's re-attach artifact.** No detach/re-attach line re-welded in the twin (R4233-R4237, R4243): the rigid rod
   pushed with `contact_dz -0.03` sinks the tip into the plate, outside the twin's contact window. A two-instance twin
   test needs the settings that re-welded (fm9's re-attach, R4239) or a twin-only contact_dz. Otherwise it will
   "fail" for a known twin reason.
4. **CPU.** Two stacks + 4 trackers + Gazebo is lab-PC work, one slot. His replans already run ~105 ms median against
   his splice (decisions 13 Nov).

What the twin can answer: the plumbing (no crossed topics), the ring's roll with two carriers while one quad proves
and is folded in, and both instances' phase timing. It cannot answer the real contact behaviour (see 3).

## Change table

| # | file:line | change | ours / his | size |
|---|---|---|---|---|
| 1 | `partner_mission.launch.py:63-66, 502-505, 987-988` | args `instance` (namespace; '' = today's names) and `drone_id`; derive state, tip, contact, take-back, mux-state, release topics from them | ours | S |
| 2 | `partner_mission.launch.py:1125-1271` | B: planner namespaced; the four YAML files as dicts; every topic typed under B's namespace after the c1f2b dict; `vehicle_id drone_<d>`; remap `/join_planner/force_drop`, `/dynamic_planner/static_target`, `/fake_moving_obstacle/state` | ours | M |
| 3 | `partner_mission.launch.py:1273-1286, 375-397` | B: backend Node built in our launch from `c1f6_moving_rendezvous.yaml` + the rig override + `_backend_overrides` as one dict (his `m2_irl_two_drone.launch.py:325-370` pattern), namespaced topics, peers = the listed carriers, `csv_path` per instance; `cooperative_preview_count` and the attached-plate list from `len(carrier_azimuths_deg)` (both instances) | ours | M |
| 4 | `partner_mission.launch.py:1053-1092` | B: ring_bridge namespaced, every output topic typed | ours | S |
| 5 | `ring_bridge.py:68` | attached-plate topic prefix as a parameter (default unchanged) | ours | S |
| 6 | `partner_rig_state.py:80-84` | accept the tip as `MotionCaptureState` (`/drone_<d>/magnet_tip_state`), default unchanged | ours | S |
| 7 | `partner_mission.launch.py:1288-1310` | MPC: `external_reference_topic`, `payload_mpc_mode_topic`, `logging_instance` typed (A: today's values, so no change) | ours | S |
| 8 | `partner_mission.launch.py:1336-1343, 1403-1405` | per-instance bag topics, `bag_path`, `result_path` | ours | S |
| 9 | `test/test_partner_mission_rig.py` (+ new) | launch-dump test: A's parameters identical to today's; every B topic under its namespace except an allow-list of shared inputs (`/payload/motion_capture_state`, `/payload/trajectory_state`, `/drone_<d>/*`); B carries the c1f2b / m1 measured-tip values | ours | M |
| 10 | `graph_attach.py`, `elrs_mux.py:156`, `dissipative_node.py:716, 920-922`, `magnet_attachment_manager.py`, `console.py`, `rig_flight.sh`, `runner_node.py` | second include and per-drone partner wiring (mux_1, gate_1, contact/proof/attached, release, take-back, console both instances, bag) | ours (our-stack study) | L |
| 11 | partner_mission sim path | twin: rig-like node set for a carrier instance (partner_rig_state, no x3 pendulum, no gz object), remapped supervisor | ours | M |
| 12 | `online_join_planner.py:2767, 2826` | (optional) hard-coded topics -> parameters; remaps do the same | his file, no behaviour change | S |
| 13 | `online_join_planner.py:9886-9958, 3765-3775` + backend peer lists | (optional) mutual awareness: an M1 moving commitment, the other quad as a peer, one C++ transit at a time | **his behaviour: Te-Jen's OK** | L |

Nothing in rows 1-11 changes how his planner, backend or MPC flies. Rows 12-13 are his code. Only 13 changes behaviour.

## Risks of a simultaneous attach

| risk | what happens | mitigation |
|---|---|---|
| Blind to each other | both routes ignore the other quad (section 2) | opposite plates, release on own radial line, per-drone takeoff gates; verify the gates in the desk test |
| A default left global | two planners on `/join_planner/reference` (or `/dynamic_planner/*`, `/join_planner/land_now`): both MPCs follow interleaved references, both backends take each other's grants | row 9's test; `ros2 topic info -v` on the desk with both up (one publisher per topic) |
| YAML-key trap | a namespaced planner or backend silently loses c1f2b, measured-tip, c1f6 and override values and flies his code defaults | dicts only (rows 2-3); the test compares B's dump with A's |
| Ring roll with two carriers | the two carriers (0 and 6) leave the ring nearly free to roll about their axis (the ring's centre of mass is only ~2-3 cm below the line joining their plates). The returning quads' plates sit on the soft axis, so one quad's LIFT_PROOF (up to 3 cm on the rig), its fold-in by our side, or its downwash lowers or raises the other quad's plate. His descent band (5 cm, 1 s debounce), match gate (5 cm, 0.20 m/s, 0.5 s) and our co-motion proof then see motion the other quad caused. Rough estimate, to be checked in the twin | descend together but prove and fold in one at a time. Our side can impose that through when it confirms contact (his bottom hold waits; watch his quiet-miss rule). A hold inside his planner needs his OK. The tilt-drop and survivor rules for two carriers are our side's (`drop_gap_deg`, `min_survivors` 2) |
| Rig laptop CPU | a second planner (30 Hz Python), MPC (acados 30 Hz), ring_bridge (50 Hz) and backend. On 13 Nov the ring_bridge states already arrived 290-395 ms late on the busy laptop, and his replans take 90-150 ms | props-off load check with both stacks up (as 14 Oct); start B a few seconds after A (his M2 launch staggers controllers by 2 s) |
| Logs collide | same-second MPC directory and planner CSV name | rows 2, 3, 7 |
| Operator | two stacks to ARM / TAKEOFF / LAND; "simultaneous" means one console action publishing to both `drone_command` topics (repeated, never `--once`); SPACE and LAND must reach both | our console (row 10) |
| His give-up or take-back | per instance: one quad taken back while the other continues | per-drone take-back and mux state (row 10) |

## Effort (hours)

| block | hours |
|---|---|
| Rows 1-9: his stack as two instances on the rig path, with tests | 6-8 |
| Desk check: both instances on fake mocap (`tools/fake_mocap.py`), param dumps, one publisher per topic, props-off CPU with both | 2-3 |
| Row 11 + the twin run plumbing for a carrier instance (our side; the world and runner belong to the twin study) | 8-12 |
| Row 10, our side (owned by the our-stack study; rough) | 10-16 |
| Row 13, mutual awareness (his code, his OK, his review) | 15-25; not before 18-19 Nov |

**His-stack part alone: about 8-11 h to a desk-verified pair of instances.**

## For Wesley / Te-Jen

1. **Te-Jen's OK** to fly his stack on a carrier (QUAD2 if the 0/90/180 cw layout puts drone 1 on plate 3; it is his
   commissioning quad) as a second instance. His code is unchanged. Tell him the two instances are blind to each
   other, so the separation is geometric.
2. **Which route the re-attach flies.** The Python transfer, as every rig detach/re-attach so far (recommended), or his
   C++ planner. With C++, each sees only the two remaining carriers. Mutual avoidance (row 13) would be new work in his
   planner and would serialise the transits.
3. **Simultaneous proofs or one at a time.** Recommended: approach and descend together, prove and fold in one at a
   time (the ring roll risk above), pending a twin check.
