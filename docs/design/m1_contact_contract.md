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
| `/join_planner/give_up` | std_msgs/Bool | his planner -> our planner | true once when he gives up (3 failed proofs, 30 s from his first DESCEND start, or 3 failed pickups) |
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
- LIFT_PROOF (new): from DESCEND or ATTACH_READY on `/magnet/contact` true (checked before his abort guards): latch
  the tip's measured offset from the contact target (plate-relative), then rise at `proof_rise_rate` (0.02 m/s) from
  the offset commanded at entry (no reference step) to `proof_rise_max` (twin 0.01, rig 0.03) above the higher of the
  target offset and that measured contact height, then hold (m1-final; R1498: a rise from a target below the plate
  ended back on the plate; R1522: a rise from a tip pressed below the target, the ring sagging, barely lifted). A
  stale measured tip: the rise from the contact target, as before (logged once). `/magnet/contact` false -> back to DESCEND
  from the lift reference (failures += 1). 3 failures or 30 s since the first DESCEND start -> give-up: publish
  `/join_planner/give_up` true, phase TAKEN_BACK.
- Pickup descent (partner launch `pickup_hold_on_attach`; m1-final, R1507): his object flag (`/tejen/object_attached`,
  in the twin his manager's weld, tip within 0.10 m of the ball centre) ends DESCEND_TO_PICKUP: MAGNET_ATTACH_WAIT
  holds the tip at its measured height and his lift starts from that hold. The stalls (R1493, R1497, R1500, R1507)
  were not missed welds: the ball welded and the descent ran ~2 s on, pushing it into the floor and buckling the
  rigid twin rod, so his lift xy gate never held. Twin only: on the rig his flag is a proximity flag (0.12 m,
  0.8 s) and his `measured_tip_tiered` path holds at a measured near-surface candidate instead.
- Pickup retry (partner launch `pickup_retry_enabled`, with `contact_attach`; m1-final): not aligned on the ball
  (attached but the tip off target, or nothing attached) for `pickup_retry_wait_s` (10 s) in MAGNET_ATTACH_WAIT, or
  the ball lost in the lift: his object magnet OFF (an attached ball drops), a vertical rise until the face clears
  the ball by `pickup_retry_clearance_m` (0.10), then APPROACH_ABOVE_PICKUP over the ball where it now lies and his
  descent again. The third failure (two retries) is the give-up, as above, held where the rise stopped.
- RING_ATTACHED (new, terminal): from any rejoin phase on `/magnet/object_attached` true: hold the lifted offset,
  plate-relative, no lead, until `/drone_3/mux_state` reads "ours", then hold where it is; no magnet commands, no
  hand-back, no phase change after our LAND.
- TAKEN_BACK (new, terminal): on `/drone_3/take_back` true or his give-up: no references needed, no restart. His
  object magnet: entered after the drop (CLEAR_DROP_ZONE to LIFT_PROOF) it is commanded OFF and held (m1-final,
  R1520: left ON from the rejoin it welded the ball lying in the ring's net while we stepped QUAD4 out); entered
  while he still carries the ball (before the drop), no object-magnet command (the ball stays on). On the rig the
  coil is one: our take-back's OFF (once the span began) releases it too.
- ring_bridge: with `full_pose_attachment` true (rig_twin), the plate offset is rotated by the full ring attitude.

## Our planner, `partner_contact: true`
- From his TAKEOFF (his phase leaves WAIT_FOR_TAKEOFF): a warm reference for drone 3 everywhere = its measured body
  (+ the ring velocity as feed-forward once it is within the warm radius of the ring); no departed hold for it.
- Ring magnet span: ON (keepalive 1 s) once his phase reaches TRANSIT_TO_REATTACH, QUAD4 is above 0.3 m and the ring
  joint reads detached; this ON bypasses the ATTACH handler (no trajectory hold, no `_start_approach`).
- Take-back (his give-up, or our LAND before the attach; also from his pickup: no ball on, no span begun, so no ring
  magnet command): `/drone_3/take_back` true; fixed hold latched at the measured position; then the departed path:
  within 1.2 m (xy) of the ring centre radially out to 1.2 m at its height (r < 0.225: out at the reserved plate's
  azimuth), else land where it is; land alone, disarmed alone; ring magnet OFF only if the span began; while
  `/tejen/object_attached` is true, drone 3's model mass + 0.1 kg.
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

## The full M1 (partner_contact with partner_attached; Wesley 10 Oct)
QUAD4 starts welded on plate 9 and carries as the fourth drone, is detached (`/fleet/detach 3`: its ring magnet OFF),
stepped out 0.5 m and up 0.3 m, released to his mission (`/partner/release`), then rejoins by the contact attach above.
World three_rigid_ground_rig_attach_welded_pickup.sdf (generated); configs m1_full_hover / m1_full_orbit.
- graph_attach: partner_contact needs partner:=true only; with partner_attached the mux gets `start_ours: true`.
- Mux `start_ours` (combined-stack O1, from branch combined-m1 8c21fe8): ours from boot; his stream only after
  `/partner/release` on its first flying command; then the attach or the take-back as above. A take-back cancels a
  pending release and blocks a later one.
- Manager, `attached_at_start` with `confirm_mode: contact_lift`: attached from t=0 (the start weld, no proof); the
  first OFF (the detach) is accepted and releases the joint; from the next ON every contact needs the lift proof, and
  an OFF after the proven attach is refused as above. A stray ON while the start weld holds starts no proof.
- Our planner: the release as before (step-out, 2 s still, `/partner/release` at 1 Hz). Then:
  - the fixed step-out hold stays while our tracker flies QUAD4; once the mux reads "partner" and his phase has left
    WAIT_FOR_TAKEOFF it gives way to the warm reference on QUAD4's measured body (critic round 1, finding 1), in the
    same tick (one reference per tick);
  - LAND before the release cancels it: QUAD4 lands with the fleet as a departed drone (out to 1.2 m, down);
  - LAND or his give-up after the release is the take-back, also before his stream flew it (QUAD4 is airborne);
    `/partner/release` stops after a take-back;
  - the ring magnet span, the proof and the fold-in as above (the joint gate reads 'detached' after our detach).
- Runner `WAIT_PICKUP_FREE timeout_s`: holds ARM until his ball weld (made at spawn on QUAD4's tip, which is also
  welded to the ring) has read 'detached' for 1 s, or his supervisor logs "pickup weld reads free"; on expiry the
  run fails and stops.
- Unchanged for the welded start: `newcomer_fault_land` stays off (`start_attached`), so a fault of our drone-3 tracker
  while ours flies QUAD4 (welded, stepping out, taken back) is a fleet emergency stop, as for any carrier; while his
  stream flies it the fleet manager does not escalate it.
