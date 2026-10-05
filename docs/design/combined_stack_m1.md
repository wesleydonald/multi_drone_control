# Combined rig stack (Tejen's mission + ours) for M1: design and critic, 2026-09-29

Status: **NOT READY to build.** The design reads the code at e2f5f3f and the critic reads it at 7393a5f. Nothing has been built or launched. Wesley asked for this "if you finish everything else", and it waits on the decisions below. It does not affect the 2026-09-30 lab day.

Wesley, 2026-09-29: the recommendations are accepted for now. Planning is paused until the 30 Sep rig data is in.

## Decisions before building (merged from the design §3 and the critic)

| # | decision | who | recommendation | source |
|---|---|---|---|---|
| 1 | What R10b is: drone 3 starts free and hovers, then rejoins (air start, `partner_attached:=false`, mux boots `partner`), or starts welded on the floor | Wesley | Free start. A welded floor start is P15 / R0732 (drone 3 is not creeped and folds in low), so it is blocked whatever the launch does. Use one config per rung. | critic 1, design 13 |
| 2 | When drone 3 starts welded: the mux boots `ours` (O1), and drone 3 gets the fleet takeoff spool instead of 0.0 | Wesley | Yes to both. Betaflight refuses to arm with the throttle high (the THROTTLE arming-disable flag), so O1 is mandatory. | design 3, critic 2 |
| 3 | DDS isolation for the combined stack: a unique ROS_DOMAIN_ID plus ROS_LOCALHOST_ONLY=1, and the launch refuses to start without them | Wesley | Yes. Tejen's laptop on the lab network publishes root `/magnet/object_attached`, which would fold drone 3 into the OCP mid-mission. | critic 4 |
| 4 | One writer for drone 3's magnet: `magnet_gate_3` follows the mux state. After a switch to `ours` it ignores OFF until ours has sent ON once. It also has an operator input. | Wesley + Tejen | Yes | design 4, critic 6, 14 |
| 5 | His aux channels 5 and 7-10 (+1 while armed): the mux zeroes them on his stream | Wesley + Tejen | Always zero them. Waiting to see what quad4's `diff all` maps is not enough. | design 5, critic 7 |
| 6 | LAND while drone 3 is on his stream: the manager refuses LAND (or relays `land_now`) while any mux is `partner`. Runbook: during MISSION the only stop is DISARM. | Wesley | Refuse | critic 8 |
| 7 | His supervisor on a stale input: `land_now` instead of a disarm (H5), a prerequisite for flight. Add a desk step that drops body 6 and then body 24 for 1 s. | Tejen | Yes | design 7, critic 3 |
| 8 | Weld distance on the rig: read `/magnet/attachment_state` with drone 3 welded on plate 3 at R0b; require < 0.04 m. Any offset is attach geometry. | Wesley | Measure first | critic 9 |
| 9 | Three lengths are measured and named separately: our `attach_cable_len`, his `magnet_drop_below_quad`, his MPC `cable_length` | Wesley + Tejen | Yes, in one rig config per rung | design 9, critic 10 |
| 10 | M1 mocap owner: our node, plus `extra_bodies` for pickup body 6, plus `partner_rig_state` | Wesley | Yes | design 1 |
| 11 | Drone 3 identity: quad4, body 14, magnet tip 24, /dev/QUAD4. His M1 flies quad4, not quad2, and he measures his kT on quad4. | Wesley + Tejen | Yes. Confirm bodies 24 and 6 in Motive. | design 2, 10, critic 16 |
| 12 | His MPC arming: supervisor `manual` for the first flights | Tejen | Yes | design 6 |
| 13 | controller_3 watchdogs during MISSION | Wesley | No code yet. Measure pose gaps in the R10a bag. | design 8 |
| 14 | M2: magnets latch OFF at boot; his router owns the mocap with `topic_prefix /tejen` | Wesley + Tejen | Yes (later) | design 11, 12 |

Built-in fixes the critic requires whichever way the decisions go:
- **Radio port lock:** the serial open adds `ioctl(TIOCEXCL)`, because pyserial's `exclusive=True` is only an advisory flock. The negative test must use his `elrs_interface_irl` and `cat /dev/QUAD4`.
- **Launch scoping:** each include is wrapped in `GroupAction(scoped=True, forwarding=False)`, because Humble leaks launch configurations between includes.
- **His parameters:** copy them into our own rig yaml with every topic written out. Never load his irl_commissioning.yaml as-is.
- **Rig backend overrides:** keep the carrier columns 0.15/0.15/0.30 (R0629).
- **Desk test:** Motive off, or the laptop off the lab network.

---

## Design

Everything here comes from reading the code at HEAD e2f5f3f. I launched nothing and edited nothing. Four files have uncommitted edits: planner_node.py, controller_mpc.py, elrs_interface.py and callback_manager_multi.py. I cite elrs_interface.py as it stands in the working tree.

---

### 1. The M1 graph

#### 1.1 One identity map, in our numbering

| our drone | quad | Motive body | magnet-tip body | radio (one writer) | M1 role |
|---|---|---|---|---|---|
| 0 | quad1 | 11 | (21) | /dev/QUAD1, `/drone_0/elrs_interface` | carrier, azimuth 150 (plate 5) |
| 1 | quad2 | 12 | (22) | /dev/QUAD2, `/drone_1/elrs_interface` | carrier, 270 (plate 9) |
| 2 | quad3 | 13 | (23) | /dev/QUAD3, `/drone_2/elrs_interface` | carrier, 30 (plate 1) |
| 3 | quad4 | 14 | **24 (UNVERIFIED)** | /dev/QUAD4, `/drone_3/elrs_interface` | welded on plate 3 (90); leaves, flies Tejen's mission, rejoins |
| ring | – | 8 | – | – | – |
| pickup object | – | 6 | – | – | – |

- Our side is fixed at motion_capture_publisher_node.py:25,33 and real_mode.py:139-143.
- Drone 3 matches his M2 quad4 (body 14, magnet 24, /dev/QUAD4, ch 6; m2_irl_two_drone.yaml:16-23).
- His M1 IRL config flies quad **12** (irl_commissioning.yaml:36), which is our drone 1. That config is not used here.
- Inside /tejen his planner's `vehicle_id 'drone_0'` is only a label (partner_mission.launch.py:131). None of his M1 nodes addresses a `/drone_k` topic by that label, so M1 has no id-map conflict as long as no node of his opens UDP or a radio.

#### 1.2 Mocap: one owner, both stacks fed

The owner is **our** `motion_capture_publisher` (real_io_launch.py:138, via real_mode.py:145-147). It binds 0.0.0.0:1511 (motion_capture_publisher_node.py:352).

His `motion_capture_publisher_irl` (bind at :319) and `m2_irl_mocap_router` (bind at :126) are **not launched**. None of the three sets SO_REUSEADDR, so a second binder fails with EADDRINUSE. That failure is the enforcement of one owner, and R0c checks it.

| topic | only publisher | source | consumers |
|---|---|---|---|
| /drone_{0..3}/motion_capture_state | our mocap node | bodies 11-14 | our trackers and dissipative node; his planner, MPC, backend and supervisor (via `state` remaps, partner_mission.launch.py:39,134,201,225,255) |
| /payload/motion_capture_state | our mocap node | body 8 | our stack, ring_bridge (ring_bridge.py:40-42) |
| /magnet_tip_pose | our mocap node | body 24, **must be typed** (real_io_launch.py:76,132) | our rig weld manager (three_attach_launch.py:797); partner_rig_state (new) |
| /pickup/motion_capture_state | our mocap node, **new `extra_bodies` param** | body 6 | partner_rig_state (new) |
| /tejen/magnet_tip_pose | partner_rig_state (new, ns /tejen) | copy of /magnet_tip_pose | his planner and manager_irl |
| /tejen/pendulum_swing_state | partner_rig_state | drone 3 (body 14) plus tip (24); the math of his `_publish_pendulum` (m2_irl_mocap_router.py:271-305) with the M1 IRL anchor z −0.030 and pair age 0.20 (irl_commissioning.yaml:42-45) | his MPC (ns-relative), his supervisor (remap :256) |
| /tejen/payload_world_state | partner_rig_state | body 6 as MotionCaptureState | his supervisor readiness (simulation_test_supervisor.py:334,366-379) |
| /tejen/pickup_object/pose | partner_rig_state | body 6 as PoseArray[0] | his planner, his manager_irl |

partner_rig_state takes over exactly the outputs of the gz `pendulum_state_publisher` (partner_mission.launch.py:77-88) and the gz object bridge (:42-44). His node parameters therefore differ from the sim only in the pickup topic and index.

#### 1.3 Radios: one writer per port

- The only process that opens /dev/QUADn is our `/drone_{n-1}/elrs_interface` (real_io_launch.py:170-175; serial comes from real_mode.py:139-143).
- His `elrs_interface_irl` is **not launched**. It opens the first /dev/ttyUSB* when `serial_port` is empty (elrs_interface_irl.py:206-209).
- Both nodes open the port non-exclusively (elrs_interface.py:184,190; elrs_interface_irl.py:200,209). Change O2/H1 adds `exclusive=True`, so a second opener raises instead of interleaving packets.

#### 1.4 ELRS path for drone 3

```
/tejen/tejen_mpc  --remap ELRSCommand-->  /drone_3/ELRSCommand_tejen --+
controller_3      --remap (:730)------->  /drone_3/ELRSCommand_diss  --+--> elrs_mux_3 --> /drone_3/ELRSCommand --> /drone_3/elrs_interface --> /dev/QUAD4
central_controller emergency fast path (main.py:142-145, :358-362) ---------------------^ (disarm only)
```

How the mux moves between states:
- **With change O1 (`start_ours`), the mux starts in `ours`.** Drone 3 is a welded carrier, which is the FLEET row of authority.md:74.
- **ours → partner:** after `/partner/release` `"running": true`, on his first armed command above idle (elrs_mux.py:180-183,199-210).
- **partner → ours:**
  - on `/join_planner/handoff_ready` True, or on our `/magnet/object_attached` True;
  - only once `_diss` is armed and above −0.99 (elrs_mux.py:185-188,212-221; handover_policy.py:29,54-79).
- **latched:** on `/fleet/abort` (elrs_mux.py:138-160).

Why O1 is needed. Today the mux boots in `partner` (elrs_mux.py:127-133) and forwards `_diss` only once it is live. On a floor start, drone 3's radio therefore sends 0.5 s timeout packets (disarmed, mid throttle; elrs_interface.py:311-316) until TAKEOFF. The first packet it forwards is then armed at hover throttle, because controller_3 has `takeoff_spool_s 0.0` (three_attach_launch.py:710). Betaflight's arm interlock would probably refuse that arm (UNVERIFIED: it depends on the FC configuration). Before O1, `partner` also forwards any `_tejen` stream before the release (:209-210). O1 closes both holes.

#### 1.5 Magnet path for drone 3: one physical magnet, one writer

Drone 3 has one magnet on AUX ch 6. The latch `/drone_3/magnet` overwrites ch 6 in every packet, idle packets included (elrs_interface.py:157,325).

Proposed: a new node **`magnet_gate_3`** (drone_magnet, ours) becomes the only code publisher on `/drone_3/magnet`. The RViz magnet row (arm_panel.cpp:237) remains as the operator's second publisher. It has two inputs:
- **ours:** `/drone_3/magnet_ours`. This is our rig manager's `magnet_latch_topic` (three_attach_launch.py:814), retargeted from `/drone_3/magnet`. The manager is driven only by `/magnet/command`:
  - the dissipative node sends OFF at drone 3's detach (dissipative_node.py:779);
  - it sends ON at the partner handoff, with a 1 Hz keepalive (:829-839, :880-886);
  - the RViz ATTACH button also publishes it (arm_panel.cpp:159).
- **partner:** `/tejen/magnet/command` from his planner: pickup ON, DROP OFF, reattach descent ON, LANDING OFF (mission_definitions.py:114-118,164-168). He re-sends it every 1 s (online_join_planner.py:6394-6412).

Gate rules:
- It follows `/drone_3/mux_state`: `partner` → partner input; `ours` or `latched` → ours input.
- On a switch it holds its last output until the newly selected input publishes. This avoids an OFF blip at the handoff, when his descent has the magnet ON (authority.md:64) and our ON is one message behind.
- It republishes at 2 Hz. Its boot value equals `drone3_magnet_initial` (ON for partner_attached; three_attach_launch.py:580-585).

What this fixes:
- His planner re-sends OFF every second while drone 3 is carrying (planner idle). Today, on a shared `/magnet/command`, that would drop drone 3 off the ring. The gate ignores it while `ours`.
- His LANDING OFF after the handoff cannot release the weld.
- The ring-weld detector stays in our manager and is armed only by our `/magnet/command`. His pickup ON never arms it, so a false fold-in is impossible.
- His object "attached" comes from `magnet_attachment_manager_irl` in /tejen with `enable_elrs_magnet_output:=false`. It publishes nothing on `/magnet/ELRSCommand` (magnet_attachment_manager_irl.py:95-98,154).

#### 1.6 Namespaces: his topics on the rig, ours untouched

| his topic | sim (partner_mission) | rig |
|---|---|---|
| object magnet command | /tejen/magnet/command (:100,:139) | same |
| object attached | /tejen/object_attached (:116,:138,:202,:223) | same, now from manager_irl (inferred attach) |
| tip pose | /tejen/magnet_tip_pose (gz pendulum) | same, from partner_rig_state |
| pendulum, payload world state | /tejen/* (gz) | same, from partner_rig_state |
| pickup pose | gz /model/payload_model/pose[1] | /tejen/pickup_object/pose[0] |
| drone command, arming | /tejen/drone_command, /tejen/drone_arming_service, /tejen/drone_arming_state_feedback (callback_manager.py:20-27) | same |
| MPC telemetry | /tejen/telemetry (no publisher) | **remap to /drone_3/telemetry** (whether his MPC uses battery_voltage is UNVERIFIED; callback_manager.py:148-149) |
| ring and carriers | /fake_payload/*, /fake_attachment_point/*, /fake_obstacles/* (ring_bridge) | same, wall clock |
| /join_planner/*, /dynamic_planner/* | root | root (we publish none on the rig: three_attach_launch.py:757) |

**No node of his may publish on:**
- /magnet/command, /magnet/object_attached, /magnet_tip_pose, /magnet/ELRSCommand;
- /drone_3/magnet, /drone_3/ELRSCommand;
- root /ELRSCommand, /motion_capture_state.

#### 1.7 Boundary signals (handoff and release)

| topic | type | publisher | subscribers | meaning |
|---|---|---|---|---|
| /partner/release | String `{"running": true}` at 1 Hz | dissipative (:258, :841-878), after DETACH, step-out and 2 s quiet | elrs_mux_3 (:107-109), his supervisor start gate (partner_mission :236 → `/partner/release`) | RELEASED_HOLD → MISSION |
| /join_planner/handoff_ready | Bool | his planner (ATTACH_READY) | elrs_mux_3 (:100-102), dissipative (:259-261) | MISSION → REJOIN, our tracker flies the last descent, ring magnet ON |
| /magnet/object_attached | Bool 30 Hz | our rig manager only | elrs_mux_3 (:96), dissipative (:401, :820-827) | weld → OCP n→n+1 |
| /drone_3/mux_state | String, latched | elrs_mux_3 (:82) | fleet manager (main.py:113-118), magnet_gate_3 | who flies drone 3 |

#### 1.8 Kill path

- Operator DISARM or ESTOP on `/fleet/command` → `_disarm_fleet(emergency=True)` (main.py:177-183, :347-362). That does three things:
  - publishes `/fleet/abort`;
  - sends one disarmed ELRSCommand straight to `/drone_{0..3}/ELRSCommand`;
  - makes the service disarms.
- elrs_mux_3 latches on `/fleet/abort`. From then on it forwards only disarmed idle commands, whichever input is live, and re-sends them every 0.05 s on a steady timer (elrs_mux.py:138-160). **This is what disarms drone 3 while his MPC keeps streaming armed commands.**
- Magnet during an abort: the gate keeps `latched` → ours, so the ring weld stays as our manager has it. The disarm command from the mux keeps the aux channels (elrs_mux.py:38-50); the latch overwrites ch 6 anyway.
- Backstops:
  - mux dead → the radio's 0.5 s watchdog (elrs_interface.py:311-316);
  - laptop dead → RX failsafe;
  - hand kill (ladder Q4): still open.
- Optional: a one-line relay `/fleet/abort` → `DISARM` on `/tejen/drone_command`, so his MPC and supervisor also know. The radio is already covered without it.
- Known gap: a controller_3 trip while `partner` is not escalated (main.py:216-224), and a tripped tracker is never live again. At handoff the mux would then keep his authority (elrs_mux.py:162-169). See decision 8.

---

### 2. Launch-file plan

#### 2.1 New top-level launch (ours)

**`src/controller_quad_load/launch/m1_rig_launch.py`** (R10b, then R11):

```
m1_rig_launch.py
  args (required, refused if empty): thrust_ratio, drone_mass, mocap_magnet_tip_body_id, partner_kt, partner_rod_m
  args (defaults): m1_rig_config:=configs/rig/m1_partner.yaml, partner_delay_s:=5.0, record:=true,
                   bag_path:=results/rig/<date>/<run>/partner_bag
  IncludeLaunchDescription(three_attach_launch.py, launch_arguments=<below>)
  TimerAction(partner_delay_s, [IncludeLaunchDescription(tejen_mission/partner_mission.launch.py,
                                launch_arguments=<below>)])
```

`configs/rig/m1_partner.yaml` is new. It is one file, read by the launch and passed to **both** includes, so the two sides cannot disagree (decision 9).

**three_attach_launch.py arguments:**
- **Fixed:** `real:=true partner:=true partner_attached:=true num_drones:=3 reserved_attach:=1 enable_approach_mpc:=false`.
- **From the canonical floor config** partner_attached_orbit_floor.yaml:17-51:
  - `load_traj orbit`, `traj_speed 0.125`, `traj_radius 0.5`, `cable_len 0.5`, `load_mass 0.86`, `target_z 0.6`;
  - `attach_azimuths_deg 150,270,30`, `attach_central false`, `diss_balanced_tensions true`;
  - `attach_handout true`, `attach_t_handout 12.0`, `attach_x_offset 0.0`, `attach_y_offset 0.25`, `attach_elev_deg 65`;
  - `attach_traj_hold_s 10`, `attach_traj_hold_mode timed`;
  - `control_mode velocity_after_handover`, `vel_ki 0`, `diss_ki_load 1.0`;
  - `kt_trim true`, `z_ki 0.4`, `reconfig_mode ocp`, `attach_cable_len` = the measured drone-3 rod;
  - `start_taut false`, `handover_elev_deg 45`, `creep_vel 0.2`.
- **Do not pass** that config's `pose_timeout_s 3.0` / `safety_ref_timeout_s 4.0`: real_mode refuses them as looser than the rig watchdogs (real_mode.py:97-100).
- `handover_settle_s`: the sim flew 1.0, the rig default is 2.0 (real_mode.py:40). Decision 9.
- **Mocap:** `mocap_drone_body_ids 11,12,13,14`, `mocap_payload_body_id 8`, `mocap_magnet_tip_body_id <24>`, `mocap_extra_bodies 6:/pickup/motion_capture_state` (new, O3). These reach real_io through the forwarding scope (real_mode.py:145-147).
- **Typed:** `thrust_ratio` (from R1), `drone_mass`, `weld_radius 0.08` (the default).

**partner_mission.launch.py arguments:**
- `real:=true` (new arg, H2);
- `plate:=3`, `carrier_azimuths_deg:=[150.0,270.0,30.0]`, `start_gate_topic:=/partner/release`;
- `object_mass_kg:=0.060`;
- `pickup_lift_height` and `minimum_search_z_m`: sim 1.40 / 1.8, **held until the cage height is measured** (P16);
- `drop_approach_clearance:=0.60`;
- `mpc_thrust_ratio:=<partner_kt>`, `cable_length:=<partner_rod_m>`, `ring_cable_len_m:=0.5` (new args);
- `supervisor_mode:=manual` (new arg, decision 6);
- `record`, `bag_path`.

#### 2.2 `partner_mission.launch.py real:=true`: node by node (his fork, H2)

This mirrors Wesley's "one graph for sim and rig" answer (ladder Q6).

| node | real:=true change |
|---|---|
| object_bridge (:42), pickup_bridges (:260), gz `release` (:265), gz `pendulum` (:77), gz `magnet` manager (:90) | **dropped** |
| every node | `use_sim_time: false` (:50, :130, :207, :232); backend `use_sim_time 'false'` (:199) |
| ring_bridge (:46) | `bridge_cable_len_m := ring_cable_len_m` (:53); elevation 45 (= rig handover_elev); plate and azimuths as typed |
| **partner_rig_state** (new, ns tejen) | subscribes /drone_3/motion_capture_state, /magnet_tip_pose, /pickup/motion_capture_state; publishes the four /tejen topics in §1.2; `anchor_z -0.030`, `max_pair_age_s 0.20` |
| **magnet_attachment_manager_irl** (ns tejen, replaces the gz manager) | IRL geometry from irl_commissioning.yaml:48-86 (attach 0.12 m, speed 0.25, dwell 0.8, contact offsets); topics as in §1.6; `pickup_object_pose_topic /tejen/pickup_object/pose`, `pickup_object_index 0`; `enable_elrs_magnet_output false`; `elrs_magnet_command_topic /tejen/magnet/ELRSCommand` (unused, belt and braces) |
| online_join_planner (:122) | `pickup_object_pose_topic /tejen/pickup_object/pose`, `pickup_object_index 0` (:136-137); physical numbers from the irl_commissioning.yaml:90-181 block instead of spanner_8mm.yaml (:126): `pickup_attach_clearance 0.03`, `payload_pickup_point_*`, `magnet_drop_below_quad` = rod (IRL 0.53, :108), `min_reference_z` to match, `c1f1_shadow_reference_timeout_s 0.20`; **kept:** `attach_ready_land_after_s 0.0` (:172; refuse any other value in real), `drop_point_source payload`, all /tejen topics |
| backend include (:191) | `extra_params_file` → new `partner_backend_overrides_rig.yaml` (object half-extents 0.043/0.043/0.011, centre −0.039); `planner_rate_hz` = his IRL rate (UNVERIFIED value); state and object topics unchanged |
| tejen_mpc (:204) | `thrust_ratio := mpc_thrust_ratio` (typed; refuse the 82.7 sim value); `cable_length := cable_length`; `pose_timeout_s 0.25`; `object_vehicle_mass_kg` = weighed quad4; remaps unchanged (:225-226) **plus `('telemetry', '/drone_3/telemetry')`** |
| supervisor (:228) | `control_mode := supervisor_mode`; `freshness_timeout_s 0.5`, `planner_status_timeout_s 3.0` (wall-clock rig values; 5/15/3000 s were for a 0.25 RTF sim); remaps unchanged (:255-257) |
| bag (:270) | drop `--use-sim-time`; add /drone_3/ELRSCommand, /drone_3/ELRSCommand_diss, /drone_3/mux_state, /drone_3/magnet, /magnet/object_attached, /partner/release, /fleet/abort |
| guards (new, like real_mode) | refuse real with `start_gate_topic` ≠ /partner/release; refuse an untyped `mpc_thrust_ratio`; refuse `attach_ready_land_after_s` ≠ 0 |

#### 2.3 Code changes, smallest first

| # | side | change | where | needs |
|---|---|---|---|---|
| O2 | ours | `serial.Serial(..., exclusive=True)` | elrs_interface.py:184,190 | none (plumbing, pytest) |
| H1 | his fork | the same, and refuse an empty `serial_port` | elrs_interface_irl.py:200,209 | tell Tejen |
| O4 | ours | real + partner_attached: refuse an empty `mocap_magnet_tip_body_id` (otherwise the rejoin weld can never fire: magnet_attachment_manager.py:437-441) | three_attach_launch.py near :575 | none |
| O3 | ours | mocap node `extra_bodies` (`id:topic` list → MotionCaptureState); real_io arg `mocap_extra_bodies` | motion_capture_publisher_node.py:321-345, :468-484; real_io_launch.py:~76,:130-135 | none (defaults unchanged, test like P13) |
| O1 | ours | elrs_mux `start_ours` param (boot `_attached=True`, policy switched); passed `partner_attached` | elrs_mux.py:66-122; three_attach_launch.py:643-649 | **Wesley** (authority rule; matches authority.md:74) |
| O5 | ours | new `magnet_gate` node (drone_magnet); manager `magnet_latch_topic` → `/drone_3/magnet_ours`; gate in three_attach when real and partner | new file; three_attach_launch.py:814 | **Wesley** (P10 magnet path) + **Tejen** (his magnet goes through our gate) |
| O7 | ours | `m1_rig_launch.py` + `configs/rig/m1_partner.yaml` | new | Wesley (attach geometry typed from the canonical config) |
| O8 | ours | `tools/fake_mocap.py --udp HOST:PORT --tip-body 24 --pickup-body 6` (Motive text packets; SIGUSR1 moves the tip onto plate 3); `tools/graph_audit.py` (expected-publisher table → PASS/FAIL) | tools | none |
| H3 | his fork | `partner_rig_state` node (the pendulum math factored out of m2_irl_mocap_router.py:271-305 into a pure function + pytest) | tejen_mission | **Tejen** (anchor −0.030 vs router −0.04) |
| H2/H4 | his fork | `real` arg as in §2.2; `partner_rig_m1.yaml`, `partner_backend_overrides_rig.yaml` | partner_mission.launch.py; config | **Tejen** |
| O6 | ours | mux zeroes channels 5 and 7-10 on the forwarded `_tejen` stream (his armed MPC sends +1 on all of them: tejen_mpc/main.py:2451,2454) | elrs_mux.py:193-197 | decision 5 (**Wesley + Tejen**), only if quad4's `diff all` maps any of them |
| O9 | ours | controller_3 watchdogs while `partner` | controller_mpc / callback_manager_multi | decision 8 (**Wesley**, safety gate) |
| H5 | his fork | supervisor FAIL → `land_now` instead of REQUEST_DISARM on the rig | simulation_test_supervisor.py:130 | decision 7 (**Tejen**) |

#### 2.4 M2 (second)

**Owner and topics.** Our node cannot give his M2 stack its per-vehicle tip pose, pendulum, `mocap/transforms` and `/m2/ring/pose`. So for M2 **his router is the single owner** (ladder P12):
- extend it to also publish our `/drone_{0..3}/motion_capture_state` (bodies 11-14) and `/payload/motion_capture_state` (8);
- add a `topic_prefix` param: its absolute `/{vehicle_id}/…` publishers (m2_irl_mocap_router.py:97-121) become `/tejen/drone_k/…`, so his drone_0 (body 12) never lands on our /drone_0 (body 11);
- our real_io gets a new `mocap:=false` (today `real_io:=false` drops radios and mocap together: dissipative_launch.py:224-229).

**His launch** (m2_irl_two_drone.launch.py, `real_partner` mode):
- namespaces `/tejen/drone_k`;
- no `elrs_interface_irl`;
- the MPC remap `ELRSCommand → /drone_{quad-1}/ELRSCommand_tejen`, derived from the identity yaml's `physical_quad` (today only arming, command and feedback are remapped: :273-277, P14).
- His launch is two-drone only (m2_irl_launch_config.py:15,49-50), so the reachable rung is M2c (two drones, hold only).

**Magnet.** One `magnet_gate_i` per drone:
- partner input: `/tejen/drone_k/magnet/command`;
- ours: the dissipative tether latch retargeted to `/drone_i/magnet_ours`;
- latches OFF at boot (decision 11).

**Ours:**
- mux `weld_topic` param, set to '' in M2 (removes the ungated `/magnet/object_attached` trigger: elrs_mux.py:96);
- a launch refusal unless `airborne_start true`, `takeoff_spool_s 0` and `auto_slot_assign false` are typed (drive_m2_handover.py:138-146 vs dissipative_launch.py:92,122);
- leave the fleet manager's fast-path ELRSCommand publishers un-remapped (dissipative_launch.py:235,333-336), so DISARM reaches the radio directly as in M1 (Wesley's word);
- an operator `/fleet/handover` publisher (an RViz button or the ladder M2c `ros2 topic pub` line).

**Blockers:**
- the fork needs his d9c69415 (P17);
- his MPC's cable 0.531 vs the tether 0.475.

---

### 3. Decisions for Wesley and Tejen

1. **M1 mocap owner.** *Recommend:* our node, plus `extra_bodies` for pickup body 6, plus partner_rig_state. It is already the rig owner for R0-R9 (P13 done), and his M1 is one drone. Put his router off until M2 (decision 12).
2. **Drone 3 identity (Wesley + Tejen; ladder Q2/Q3).** *Recommend:* drone 3 = quad4 = body 14 = /dev/QUAD4 = magnet body 24; his M1 flies quad4, not quad2. Confirm in R0b that Motive bodies 24 and 6 exist.
3. **Drone 3's mux starts `ours` (O1, Wesley).** *Recommend:* yes. It removes the floor-start arm-interlock risk and any `_tejen` forwarding before the release.
4. **One writer for drone 3's magnet: `magnet_gate_3` selecting by mux state (O5, Wesley + Tejen).** *Recommend:* yes. His object commands are honoured only in MISSION, and his OFFs can never release the ring weld. His manager_irl runs with ELRS output off.
5. **His aux channels 5 and 7-10 (+1 when armed) (Wesley + Tejen).** *Recommend:* read quad4's `diff all` at R0b (notes_for_tejen.md:9-11). If any of them maps to a Betaflight mode, the mux zeroes them on his stream (O6); otherwise pass them through.
6. **Arming his MPC.** *Recommend:* supervisor `manual` for R10b and the first R11: Tejen presses ARM and TAKEOFF after `/partner/release`. Our tracker holds the step-out point indefinitely, and his armed idle (throttle −1) does not trigger the switch. Go `automatic` (sim) once it has flown.
7. **His failure mode mid-mission (Tejen).** His supervisor's `_stop` disarms his MPC (simulation_test_supervisor.py:130), which drops drone 3 mid-air on his stream. *Recommend:* on the rig, his FAIL requests `land_now`; our DISARM stays the kill.
8. **controller_3 watchdogs during MISSION (Wesley).** A pose gap over 0.25 s at his 1.8-1.9 m transit trips our tracker, and the rejoin then fails safe (the mux keeps his authority). *Recommend:* no code yet. Measure drone-3 pose gaps at his transit height in the R10a bag; add O9 only if trips show.
9. **Values both sides must type, from one file (Wesley + Tejen).** *Recommend:* `configs/rig/m1_partner.yaml` with:
   - plate 3 and carriers 150/270/30;
   - `attach_y_offset +0.25`;
   - gate /partner/release;
   - drone-3 rod: our `attach_cable_len` = his `cable_length` = `magnet_drop_below_quad` (measured once);
   - `ring_cable_len_m` = our `cable_len`;
   - `handover_settle_s`: rig 2.0, or the 1.0 the sim floor flew. *Recommend* 1.0, since the rig should fly the sim claim.
10. **His kT on quad4 (Tejen).** *Recommend:* he measures it in R10a on quad4 with his own definition. It is not our 24 by assumption (his IRL default is 22 plus UKF: tejen_mpc/main.py:497).
11. **M2 magnet latches at boot (Wesley).** *Recommend:* OFF for M2. Today the code latches all four ON (dissipative_launch.py:227-229), which contradicts the readiness review (g7_readiness_review_2026-09-29.md:327). His join owns the magnet until `/fleet/handover`, through the gate.
12. **M2 mocap owner (Wesley + Tejen; P12).** *Recommend:* his router with `topic_prefix /tejen`, plus our topics, with our `mocap:=false`.
13. **Floor start.** R11 remains blocked on P15 / GOALS Q9 (R0732: drone 3 is not creeped and folds in low) whatever this launch does. *Recommend:* build and desk-test this launch for R10b (the hover rejoin) first.

---

### 4. R0c: props-off desk test against fake mocap

**Set-up**
- Laptop, all four TX modules plugged in (/dev/QUAD1-4). Drones 0-2 powered off; drone 3 powered **props off** for step 7.
- No Gazebo or SIL (`tools/clean_slate.sh` first).
- Needs O1-O5, O7, O8, H2-H4.
- One registry row (D-series, desk, like D0003).

**Steps**
1. `python3 tools/fake_mocap.py --udp 127.0.0.1:1511 --num-drones 3 --attach --azimuths-deg 150,270,30 --drone-z 0.08 --tip-body 24 --pickup-body 6`. Drone 3 sits on plate 3, the tip away from the plate.
2. `ros2 launch bringup m1_rig_launch.py thrust_ratio:=24 drone_mass:=<w> mocap_magnet_tip_body_id:=24 partner_kt:=<k> partner_rod_m:=<r>`.
3. **Graph audit** before anything is injected: `python3 tools/graph_audit.py m1_rig`. It wraps `ros2 topic info -v` for each topic, then:
   - `ss -ulpn 'sport = :1511'`;
   - `for n in 1 2 3 4; do lsof "$(readlink -f /dev/QUAD$n)"; done`;
   - `lsof /dev/ttyUSB*`.
4. Negative tests:
   - start `ros2 run tejen_mission m2_irl_mocap_router`: it must die with EADDRINUSE while our poses keep flowing;
   - start a second `elrs_interface` with `serial_port:=/dev/QUAD4`: it must fail to open.
5. **Run 1 (DISARM on his stream):**
   1. RViz ARM, then TAKEOFF (props off; trackers go above idle).
   2. `ros2 topic pub --once /magnet/command std_msgs/String "data: OFF"`, then `ros2 topic pub -r 1 /partner/release std_msgs/String '{data: "{\"running\": true}"}'`.
   3. Tejen ARMs and TAKEs OFF his MPC through /tejen. If his planner gives no reference on static mocap (UNVERIFIED), stop his MPC and use the stand-in: `ros2 topic pub -r 30 /drone_3/ELRSCommand_tejen interfaces/msg/ELRSCommand "{armed: true, channel_2: -0.5}"`.
   4. Check the mux goes to `partner`.
   5. Watch `/drone_3/magnet` follow his `/tejen/magnet/command` (his planner's 1 Hz re-send, or one injected ON then OFF).
   6. RViz **DISARM**.
6. **Run 2 (handoff):** relaunch (the abort latch holds until restart), repeat 5.1-5.4, then `ros2 topic pub --once /join_planner/handoff_ready std_msgs/Bool "data: true"`. Then inject `/tejen/magnet/command OFF`.
7. **Run 3 (weld):** relaunch, repeat 5.1-5.4, then SIGUSR1 (tip onto plate 3) and press RViz ATTACH (`/magnet/command ON`).

**Pass criteria**

| # | criterion |
|---|---|
| P1 | exactly one socket on UDP 1511, owned by `motion_capture_publisher` |
| P2 | each /dev/QUADn is open by exactly one PID, `/drone_{n-1}/elrs_interface`; no other /dev/ttyUSB* is open; both negative tests in step 4 fail as expected |
| P3 | publishers: <br>• `/drone_{0,1,2}/ELRSCommand` = {controller_i, central_controller} <br>• `/drone_3/ELRSCommand` = {elrs_mux_3, central_controller} <br>• `/drone_3/ELRSCommand_tejen` = {/tejen/tejen_mpc} <br>• `/drone_3/ELRSCommand_diss` = {controller_3} <br>• `/drone_3/magnet` = {magnet_gate_3, rviz2} <br>• `/drone_{0..2}/magnet` = {dissipative_controller, rviz2} <br>• `/drone_3/magnet_ours` = {magnet_attachment_manager} <br>• `/magnet/command` = {dissipative_controller, rviz2} <br>• `/magnet/object_attached` = {magnet_attachment_manager} <br>• `/magnet_tip_pose` = {motion_capture_publisher} <br>• `/tejen/magnet/command` = {online_join_planner} <br>• `/tejen/object_attached` = {/tejen/magnet_attachment_manager_irl} <br>• `/tejen/{magnet_tip_pose, pendulum_swing_state, payload_world_state, pickup_object/pose}` = {/tejen/partner_rig_state} <br>• `/join_planner/handoff_ready` = {online_join_planner} <br>• `/partner/release` = {dissipative_controller} <br>• **zero** publishers on /magnet/ELRSCommand, /ELRSCommand, /motion_capture_state, /magnet/attachment_state from any /tejen or partner node |
| P4 | at boot `/drone_3/mux_state` = `ours`; the first `/drone_3/ELRSCommand` after ARM is armed with throttle −1 (armed idle), never armed above idle as the first armed packet (O1) |
| P5 | Run 1: the mux goes `partner` on the first armed above-idle `_tejen` after the release; `/drone_3/magnet` equals his last command within 0.5 s, and is ON from boot until our OFF |
| P6 | Run 1 DISARM: within 0.1 s, `/drone_3/mux_state` = `latched`, and every `/drone_3/ELRSCommand` has `armed: false`, `channel_2: -1` at ≥ 15 Hz for 10 s while `_tejen` is still armed; drone 3's Betaflight is disarmed within 0.5 s (`/drone_3/telemetry` mode; props off) |
| P7 | Run 2: the mux goes `ours` within one `_diss` period (≤ 0.05 s) of the handoff with a live controller_3; no gap > 0.1 s in `/drone_3/ELRSCommand` (bag); `/drone_3/magnet` goes ON (from `/drone_3/magnet_ours`) with no OFF sample at the switch; the injected `/tejen/magnet/command OFF` leaves it ON |
| P8 | Run 3: `/magnet/object_attached` True within 0.15 s dwell + one 30 Hz tick of ATTACH with the tip at the plate; mux `ours` on the next live `_diss`; no gap > 0.1 s |
| P9 | the node log has no `use_sim_time` true and no `/clock` subscriber; `ros2 param get /tejen/tejen_mpc thrust_ratio` equals the typed value, not 82.7; `/online_join_planner attach_ready_land_after_s` = 0.0 |

R0c checks the wiring only. With static mocap, no DETACH, step-out or fold-in can happen, so the mission sequence stays the sim's evidence (R0724 and the R10b sim config m1_rejoin_hover_u_t01_bb.yaml).
---

## Critic

**Verdict: NOT READY.** Several items below must be settled before building, most importantly 1–5. I read the code at HEAD 7393a5f; the working tree is clean, and the four "uncommitted" files named in the design are now committed in 7393a5f. I launched nothing and edited nothing.

**Confirmed correct**
- One mocap owner: none of the four binders sets SO_REUSEADDR (ours motion_capture_publisher_node.py:352; his m2_irl_mocap_router.py:126 and motion_capture_publisher_irl.py:319).
- No node that partner_mission launches imports serial or socket. Only his mocap publishers, the router and his elrs_interface* do.
- The fleet manager manages drone 3 (three_attach_launch.py:474 sets `num_drones n+1`). Its emergency fast path reaches `/drone_3/ELRSCommand` directly (main.py:142-145, 358-362).
- The mux abort latch re-sends every 0.05 s on a steady-clock timer (elrs_mux.py:146-152).
- His MPC never reads `battery_voltage`, so the telemetry remap is harmless.
- `attach_ready_land_after_s` is 0.0 (partner_mission.launch.py:172).

**Findings**

1. **High. The rung and the arguments disagree, and "R10b first" does not avoid P15.**
   - The ladder defines R10b as "ring carried by 3" (g7_real_world_ladder.md:51). The design's fixed arguments are `partner_attached:=true` plus the orbit floor config (partner_attached_orbit_floor.yaml:17-51, `load_traj orbit`).
   - R10b's sim evidence, m1_rejoin_hover_u_t01_bb.yaml, is `load_traj hover`, an air start (`start_taut true`), with `attach_t_start_new 0.1` and `attach_blend_balanced true`.
   - On the rig, `partner_attached` always means a floor start with drone 3 welded. That is exactly P15 / R0732, so decision 13 is wrong.
   - Fix: Wesley decides what R10b is.
     - If drone 3 starts free: `partner_attached:=false`. The mux must then boot `partner`, so O1 must be off for that rung.
     - If R10b stays partner-attached: P15 blocks it too.
     - Either way, use one config file per rung (hover arguments for R10b), not one m1_partner.yaml.

2. **High. Drone 3 takes off from the floor with no spool.**
   - controller_3 has `takeoff_spool_s: 0.0` (three_attach_launch.py:710). The carriers use the fleet value (default 0.5, :145).
   - With O1 the FC arms at idle, then at TAKEOFF jumps to hover throttle in one packet while it is welded to the ring.
   - The spool 0 was meant for a mid-air takeover. The rejoin takeover does not pass through TAKEOFF, so a spool on drone 3 changes nothing there.
   - Fix: when `partner_attached`, pass `f('takeoff_spool_s')` to controller_3.
   - Also: Betaflight refusing to arm with throttle high is standard behaviour (the THROTTLE arming-disable flag), not just a guess. That makes O1 mandatory, not optional.

3. **High. His supervisor can drop drone 3 mid-air on a stale input.**
   - `_inputs_ready` requires all of these to be fresh (simulation_test_supervisor.py:365-378):
     - `/tejen/payload_world_state` (body 6);
     - `/tejen/pendulum_swing_state` (bodies 14 and 24 paired within 0.20 s);
     - `/dynamic_planner/transfer_status`.
   - If any goes stale while RUNNING, `_stop` requests a disarm (:199-200, :120-135). His MPC then streams `armed:false`, and the mux forwards it while in `partner`.
   - The design tightens `freshness_timeout_s` to 0.5 s. Body 6 is likely to be occluded under the magnet or lost after a free-air DROP. Body 24 is likely to be occluded near the pickup object or the ring.
   - Static fake mocap cannot show any of this.
   - Fix:
     - make H5 a prerequisite: on the rig a FAIL requests `land_now` and never disarms;
     - or have partner_rig_state hold body 6 after the drop;
     - add a desk step: fake_mocap drops body 6 and then body 24 for 1 s while in `partner`. Pass means drone 3 stays armed.

4. **High. Nothing isolates DDS.**
   - No ROS_DOMAIN_ID or ROS_LOCALHOST_ONLY is set in the launches, the tools or the rig docs.
   - If Tejen's own laptop runs irl_state_pipeline on the lab network, its root topics join our graph (irl_commissioning.yaml:47-58: `/magnet/command`, `/magnet/object_attached`, `/magnet_tip_pose`, `/magnet/ELRSCommand` with ELRS output on).
   - A `/magnet/object_attached` True during MISSION:
     - flips the mux to our tracker (elrs_mux.py:96, 175-178);
     - folds drone 3 into the OCP while it is metres from the ring (dissipative_node.py:401, 820-827).
   - Fix:
     - set a unique ROS_DOMAIN_ID plus ROS_LOCALHOST_ONLY=1 for the combined stack, and have m1_rig refuse to start without them;
     - graph_audit fails on any node or publisher not in its expected list;
     - run R0c with Tejen's laptop on the network.

5. **High. O2 is only an advisory lock, and the negative test cannot fail.**
   - pyserial 3.5 implements `exclusive=True` as `fcntl.flock(LOCK_EX|LOCK_NB)` (serialposix). It only stops other openers that also use flock.
   - These would still open the port and interleave packets: his `elrs_interface_irl` from his own repo (not the fork, so no H1), `cat`/`screen`, or Betaflight Configurator.
   - The step-4 negative test opens a second copy of *our* node, so it passes trivially.
   - Fix: also issue `ioctl(fd, TIOCEXCL)` after open, which the kernel enforces for non-root openers. Run the negative test with his `elrs_interface_irl` (empty `serial_port` and `/dev/QUAD4`) and with `cat /dev/QUAD4`.

6. **Medium-high. The handoff magnet rule has a gap, and desk Run 2 fails P7 by construction.**
   - Run 2:
     - `_partner_handoff_cb` returns unless `attach_pending` is set (dissipative_node.py:829-830);
     - only the release tick after a DETACH sets it (:866-869);
     - Run 2 injects the release from the command line with no DETACH, so no ON is ever sent;
     - the gate then passes the manager's OFF keepalive, and P7 fails.
   - On the rig:
     - the ON goes out on the next 10 Hz plan tick (`_partner_magnet_sent=None` at :839, keepalive at :880-886; PLANNER_HZ in planner_node.py:56);
     - the mux switches on the next live `_diss`, within about 20 ms;
     - the manager re-sends OFF at 2 Hz (magnet_attachment_manager.py:590);
     - so about 1 in 5 handoffs gives an OFF of up to 0.1 s. "Hold until the new input publishes" does not prevent it.
   - Fix:
     - after a partner→ours switch the gate ignores OFF from ours until ours has sent ON once;
     - or `_partner_handoff_cb` publishes ON synchronously;
     - Run 2 must fly a real `/fleet/detach 3`, with fake_mocap moving drone 3 to the step-out point, or say that it injects the ON.

7. **Medium. Make O6 unconditional.**
   - His MPC sends +1 on channels 5-10 whenever armed (tejen_mpc/main.py:2451, 2454). That is his magnet fan-out for an AUX he never positively identified (irl_commissioning.yaml:79-82), not something his flight needs.
   - On quad4, AUX3/5/6/7 carry our mode map, and the change would land at the exact handover instant.
   - Fix: the mux sets channels 5 and 7-10 of the forwarded `_tejen` stream to 0, whatever `diff all` shows. Channel 6 is already overwritten by the latch.

8. **Medium. LAND does not stop drone 3 while it is on Tejen's stream.**
   - `_land_fleet` leads to `/fleet/landed`, then `_disarm_fleet(emergency=False)` (main.py:329-345, 352). That is service disarms only: no `/fleet/abort` and no fast path.
   - controller_3's disarm is not forwarded in `partner` (elrs_mux.py:212-221). The ring lands and drone 3 keeps flying his mission.
   - Fix: the manager refuses LAND (or relays `/join_planner/land_now`) while any mux is `partner`. Runbook: DISARM is the only stop during MISSION.

9. **Medium. The rig weld distance has never been measured.**
   - The manager declares a weld when the tip marker (body 24) is within 0.08 m of body 8's origin + (0, 0.25) (three_attach_launch.py:797-807).
   - On the rig the marker sits above the contact face (his −0.030 at irl_commissioning.yaml:67-70), and body 8's pivot relative to the plate top is unknown.
   - fake_mocap places the tip exactly on the plate, so Run 3 passes whatever the real geometry is.
   - Fix: at R0b, with drone 3 physically welded on plate 3, read the live distance on `/magnet/attachment_state` and require it to be under 0.04 m. Any offset parameter is attach geometry and needs Wesley's word.

10. **Medium. Decision 9 treats three different lengths as one.**
    - Our `attach_cable_len` is body centre to tip (sim 0.49).
    - His `magnet_drop_below_quad` is body origin to the magnet *marker*, and his config says it is "intentionally separate from the MPC effective pendulum-length" (irl_commissioning.yaml:103-108).
    - His MPC `cable_length` is that effective pendulum length, measured from the −0.030 anchor.
    - Fix: measure all three and name each one separately in the config.

11. **Medium. Launch configurations leak between the two includes.**
    - Humble's `IncludeLaunchDescription.execute` only emits SetLaunchConfiguration and does not scope (~/ros2_humble/src/ros2/launch/launch/launch/actions/include_launch_description.py).
    - About 110 three_attach configurations, real_mode's rewritten values and the global `SetParameter(use_sim_time)` (three_attach_launch.py:390) are all visible to partner_mission when it starts 5 s later.
    - Any argument with the same name (the new `real`, or `record`/`bag_path` if three_attach ever gains them) takes three_attach's value, and partner_mission's default is silently skipped.
    - Fix: wrap each include in `GroupAction(scoped=True, forwarding=False)` with explicit arguments.

12. **Medium. His parameter file must not be loaded as-is.**
    - If irl_commissioning.yaml is passed as a params file, its root topics, and for manager_irl ELRS output on across channels 5-10, apply wherever the dict does not override them (:47-86, :90-101).
    - The `online_join_planner:` key matches the root-namespace planner by name. The `fake_cooperative_transport_world:` key matches ring_bridge's node name (partner_mission.launch.py:47).
    - §1.6 omits `attachment_state_topic`. The sim sets it to `/tejen/...` (:117), but manager_irl's default is `/magnet/attachment_state` (magnet_attachment_manager_irl.py:44), which ours also publishes.
    - Fix: copy the values into partner_rig_m1.yaml with every topic written out explicitly. Add `/magnet/attachment_state` to P3's zero-publisher list.

13. **Medium. The rig backend overrides must keep the carrier columns.**
    - Replacing partner_backend_overrides.yaml wholesale drops `cooperative_body_half_*` 0.15/0.15/0.30. Those columns exist because in R0629 the rod swept into carrier 0.
    - Fix: the rig file is the sim file plus the new object extents.

14. **Low-medium. The magnet gate hides two things.**
    - The gate's 2 Hz republish overwrites the RViz drone-3 magnet row (arm_panel.cpp:237) within 0.5 s. Operator control is lost silently, yet P3 counts rviz2 as a working writer.
    - The gate must subscribe to `/drone_3/mux_state` with MUX_STATE_QOS (TRANSIENT_LOCAL, elrs_mux.py:33-34), or it misses the boot state.
    - Fix: give the gate an operator input that wins until the next mux state change.

15. **Low. Desk test set-up.**
    - Motive must be off or the laptop off the lab network. Otherwise real UDP packets on 1511 mix with fake_mocap's.
    - Check that `m2_irl_mocap_router` exits on EADDRINUSE rather than retrying. Re-check P1 after the negative tests.

16. **Low. The ladder is out of step with the design.**
    - R10b lists P12, the unified router, as a prerequisite (g7_real_world_ladder.md:346). Decision 1 replaces it for M1, so update the ladder row.
    - R10a flies quad2 / body 12 / magnet 22 (:338-342). It proves nothing about quad4, so his kT and pendulum anchor need a quad4 run of his.

17. **Low. P9 should check more of his sim values.** Also confirm that `enable_thrust_ratio_ukf` (sim True, partner_mission.launch.py:209) and `object_vehicle_mass_kg` (0.643, the sim X3 mass, :222) are deliberately set for the rig.

**What the R0c desk test would miss:** findings 3, 4 (unless Tejen's laptop is on the network), 5, 9, 11 (while no argument names collide) and 12 (if typed correctly). It also gives false failures on P7 (finding 6) and a coin-flip on P7's "no OFF at the switch" clause.

**Decisions for Wesley:** 1 (what R10b is), 2 (drone 3's spool), 7 (O6 unconditional), 8 (LAND while `partner`), 9 (weld geometry), and whether H5 becomes a blocker (3).