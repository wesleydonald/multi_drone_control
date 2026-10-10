# Combined M1 contact attach: the interface contract between the three builds (10 Oct)

Card: /home/wesley/multi_drone_control/docs/experiments/2026-10-10_combined_m1_contact.md (rev 4). Everything is behind
switches that default OFF: with them off, every node behaves exactly as today. Branch m1-subtest.

## Switches
- Our launch arg `partner_contact` (bool, default false; bringup common.yaml + profiles KEPT args). graph_attach passes it:
  - to the dissipative planner as parameter `partner_contact`;
  - to the magnet manager as `confirm_mode: contact_lift` (else `weld`, today's check);
  - to the mux as `handoff_topic: ''` and `take_back_topic: /drone_3/take_back`;
  - to the drone-3 tracker as `warm_follow_applied: true`.
- His planner parameter `contact_attach` (bool, default false), set by his partner launch arg `contact_attach:=true`.

## Topics (new unless marked existing)
| topic | type | writer -> readers | meaning |
|---|---|---|---|
| `/magnet/contact` | std_msgs/Bool | our manager -> his planner, our planner | true: contact detected (latched c0, z0); false: proof failed / reset |
| `/magnet/object_attached` (existing) | std_msgs/Bool | our manager -> mux, our planner, his planner | true: the attach is CONFIRMED (proof passed); latched until a magnet OFF |
| `/magnet/attach_proof` | std_msgs/String | our manager -> logs | JSON at 10 Hz: state, drift_m, plate_rise_m, body_to_tip_m, tries |
| `/join_planner/give_up` | std_msgs/Bool | his planner -> our planner | true once when he gives up (3 failed proofs or 30 s from his first DESCEND start) |
| `/join_planner/phase` (existing) | std_msgs/String | his planner -> ours | his phase name |
| `/drone_3/take_back` | std_msgs/Bool | our planner -> mux, his planner | true: QUAD4 goes to our tracker now (no attach) |
| `/drone_3/mux_state` (existing) | std_msgs/String | mux -> all | "partner" / "ours" |
| `/magnet/command` (existing) | std_msgs/String | our planner -> our manager | the RING magnet: ON / OFF |
| `/tejen/object_attached` (existing) | std_msgs/Bool | his object manager -> ours | his ball is on |
| `/payload/detachable_joint_state` | gz -> ROS (bridge in graph_attach, sim) | -> our planner | the ring weld joint: the span ON waits for "detached" after spawn |
| `/pickup/detachable_joint_state` | gz -> ROS (bridge in his launch) | -> his launch / planner | the ball weld: his takeoff waits until it reads free after spawn |

## Our manager, `confirm_mode: contact_lift` (all timers on the node clock: sim time in the twin)
- Contact: magnet ON, tip within `attach_radius` of the plate point (full ring pose + offsets + `object_z_offset`),
  relative speed <= `attach_speed_threshold`, for `contact_dwell_s` (0.15). Latch c0 = mean tip-to-plate vector over
  the dwell, z0 = plate point z. Publish `/magnet/contact` true. Twin (`sim_catch` true, default in sim): make the gz
  joint now (the simulated catch); `sim_catch` false = the `_nocatch` arm.
- Proven, for `proof_dwell_s` (0.3): |MA0.1s(tip-to-plate) - c0| <= `proof_d_ok` (0.003) AND plate z - z0 >=
  `proof_h_plate` (0.003) AND (rig only, `proof_taut_len` > 0) body-to-tip >= `proof_taut_len`. Then
  `/magnet/object_attached` true (latched).
- Failed: drift >= `proof_d_fail` (0.008) for `proof_fail_dwell_s` (0.12), or no decision within `proof_timeout_s`
  (3.0) of the contact: `/magnet/contact` false; twin: release the joint; back to watching for contact.
- Arbiter: an OFF after `object_attached` true is refused (logged); an OFF before it resets contact/proof and nothing
  is declared attached until a new ON.

## His planner, `contact_attach: true`
- DESCEND_TO_ATTACHMENT to the contact target (launch: rig_twin `bridge_attach_z_m` 0.032, `final_attach_offset_z` 0).
- ATTACH_READY: no `handoff_ready`, lead 0, match 0.025 m / 0.08 m/s, abort 0.05 m / 0.15 m/s.
- LIFT_PROOF (new): from DESCEND or ATTACH_READY on `/magnet/contact` true (checked before his abort guards): rise at
  `proof_rise_rate` (0.02 m/s) up to `proof_rise_max` (twin 0.01, rig 0.03) above the contact target, plate-relative,
  then hold. `/magnet/contact` false -> back to DESCEND (failures += 1). 3 failures or 30 s since the first DESCEND
  start -> give-up: publish `/join_planner/give_up` true, phase TAKEN_BACK.
- RING_ATTACHED (new, terminal): from any rejoin phase on `/magnet/object_attached` true: hold the lifted offset,
  plate-relative, no lead, until `/drone_3/mux_state` reads "ours", then hold where it is; no magnet commands, no
  hand-back, no phase change after our LAND.
- TAKEN_BACK (new, terminal): on `/drone_3/take_back` true or his give-up: no references needed, no object-magnet
  commands (the ball stays on), no restart.
- ring_bridge: with `full_pose_attachment` true (rig_twin), the plate offset is rotated by the full ring attitude.

## Our planner, `partner_contact: true`
- From his TAKEOFF (his phase leaves WAIT_FOR_TAKEOFF): a warm reference for drone 3 everywhere = its measured body
  (+ the ring velocity as feed-forward once it is within the warm radius of the ring); no departed hold for it.
- Ring magnet span: ON (keepalive 1 s) once his phase reaches TRANSIT_TO_REATTACH, QUAD4 is above 0.3 m and the ring
  joint reads detached; this ON bypasses the ATTACH handler (no trajectory hold, no `_start_approach`).
- Take-back (his give-up, or our LAND before the attach): `/drone_3/take_back` true; fixed hold latched at the measured
  position; then the departed path: within 1.2 m (xy) of the ring centre radially out to 1.2 m at its height
  (r < 0.225: out at the reserved plate's azimuth), else land where it is; land alone, disarmed alone; ring magnet OFF
  only if the span began; while `/tejen/object_attached` is true, drone 3's model mass + 0.1 kg.
- Lifecycle gates: `newcomer_lifecycle` / `newcomer_fault_land` / `departed_ground_idle` stay on under
  `partner_contact`; `_newcomer_arming_cb` acts only while `/drone_3/mux_state` reads "ours".
- On `/magnet/object_attached`: the existing fold-in, with QUAD4's starting share seeded from the measured pull
  (not `attach_t_start_new` 0.1); `_attach_ocp` drops any departed entry for the drone.

## Our drone-3 tracker, `warm_follow_applied: true`
- While `/drone_3/mux_state` != "ours": `_applied_u` (all four channels) and `last_cmd_throttle` come from the mux
  OUTPUT `/drone_3/ELRSCommand` (throttle through `_realised_model_throttle` from (ch2+1)/2, the model units);
  `_applied_cable_vec`, the thrust history and the integrators are frozen; the takeoff spool is marked done once it
  has a reference.
- At the switch to "ours": x0 from the measured state, the solver re-seeded (first_solve), the heading datum
  re-latched.

## Mux
- `take_back_topic` true -> switch to ours (latched, the live-takeover check still applies); `handoff_topic ''` = not
  wired.

## Runner
- `WAIT_REWELD` takes an optional timeout (s): on expiry it logs and the script goes on (to LAND).

## The rig (branch m1-rig, 11 Oct): `real_control_launch.py mode:=attach partner:=true partner_contact:=true`
Everything below exists only with real + partner_contact (the rig launches flown today are unchanged: launch dumps
of the Wednesday sheet lines, the frozen launch gate and the twin claim configs are identical before and after).
- **DDS**: the launch refuses unless `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID` are set (combined-stack critic 4).
- **Contact radius**: `weld_radius` (> 0, refused at 0), at `weld_point_z` (the 09:00 calibration, attach_m1.yaml);
  `proof_taut_len` typed from the 09:00 tip record (required > 0: on the slack tether an uncaught magnet resting on
  a rising plate would otherwise read as attached).
- **One coil** (O5): the manager's latch goes to `/drone_3/magnet_ours`; `magnet_gate_3` alone writes
  `/drone_3/magnet` (QUAD4's radio latch): his `/tejen/magnet/command` until the first ON on `/drone_3/magnet_ours`
  (our span, or any `/magnet/command` ON), then ours only, for good. Before it our OFF keepalive is dropped (a
  take-back before the span keeps his ball on); after it his OFFs are dropped. `/drone_3/magnet_gate` (latched) says
  which.
- **His aux channels** (O6): the mux zeroes channels 5 and 7-10 of his stream (`zero_partner_aux`).
- **Mocap** (O3): our mocap node routes his ball, `mocap_extra_bodies` (default under partner_contact
  `6:/pickup/motion_capture_state`), next to QUAD4 (14) and its marker (24, `/magnet_tip_pose`).
- **His nodes** (`partner_nodes:=true`, 5 s after ours, own launch scope): `tejen_mission partner_mission.launch.py
  real:=true geometry:=rig contact_attach:=true contact_target_z:=<weld_point_z>` plus `partner_args` ('k:=v ...').
  `partner_rig_state` turns our mocap into his `/tejen/magnet_tip_pose`, `/tejen/pendulum_swing_state`,
  `/tejen/pickup_object/pose`, `/tejen/payload_world_state`. His MPC: `/tejen/drone_command` (ARM, TAKEOFF by hand),
  out on `/drone_3/ELRSCommand_tejen`. No supervisor unless `partner_args:='supervisor:=true'` (stops land).
- **Order** (require_fc_armed checks QUAD4's own radio too): his ARM (QUAD4 armed idle) -> `fleet arm` ->
  `fleet takeoff` -> the ring at height -> his TAKEOFF. A take-back before our TAKEOFF cannot switch (our drone-3
  tracker is not flying): QUAD4 stays on his stream.
- **The lift margin on the rig**: his LIFT_PROOF rises from his contact TARGET (`final_attach_offset_z`, rig
  -0.01), not from the height where the contact was seen (z0), by at most `proof_rise_max` (rig 0.03): his reference
  ends 2 cm above the face, of which the first ~1 cm takes up the tether's slack. (The twin's 1 cm cap from a target
  1 cm low only brings the tip back to the plate: R1498, three caught contacts timed out.) Rising from z0 is the
  proper fix; it changes his planner and the claim tree, so it waits for Wesley.
