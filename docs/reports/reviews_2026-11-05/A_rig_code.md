# Review A: rig code for Wed 11 / Thu 12 Nov (week8 @ bf52a8a, diff 97da92d..bf52a8a)

Read-only review, 5 Nov 22:26-22:50. Tests run (nice, before 00:30): 94 targeted unit tests (tracker carrier_down_alone,
off_ring_checks, ground_idle; planner carriers_down_alone, land_no_floor, touchdown_gate/_stale; dissipative
carriers_take_back_busy; tejen takeoff_gate/_from_air/_hold; fleet_manager down_alone) and 6 related launch tests:
all pass.

## Finding 1 (most severe; mechanism CONFIRMED by code and the k1ob_2 log, trigger PLAUSIBLE): QUAD4's tracker can beat the ring drop and turn it into a fleet-wide disarm in the air

**Where.** `src/bringup/bringup/graph_attach.py:877` gives QUAD4's tracker only `tracker_land_options` (no
`drop_on_loss`/`drop_tilt_deg`), while the carriers get `tracker_drop_options` (:531). In
`src/tracker/tracker/tracker_node.py:969-983` `_payload_checks_down()` stands the 60 deg ring envelope down only for
`_off_ring` (mux 'partner'), `/fleet/payload_dropped`, or the tracker's *own* drop request. The tracker never subscribes
to `/fleet/drop` (it only publishes it, :1022). QUAD4's tracker keeps `max_payload_tilt_deg` 60 (no
`payload_envelope_options` passed). The envelope debounces 3 control ticks (`utility_objects/safety.py:70`, ~40-60 ms at
50 Hz).

**Failure.** On the drop-55 lines (nrs x3 claim, nrs0, nrtf, nro, nros: `drop_tilt_deg 55`) QUAD4's mux is 'ours' for
most of the flight: welded on the ring before DETACH, stepping out and waiting until his stream takes it (the T leans
21-31 deg then, sq8_2), and on the ring again after the re-weld. If the ring passes 55 deg fast (a magnet letting go:
k1ob_2 went 7 -> 54 deg in 0.2 s), the carriers request the drop and stand down at once, but the planner acts on
`/fleet/drop` only between its solves: measured request-to-DROP latency on the rig 4-5 Nov is 34-185 ms (k1m_3 34,
k1ob_2 60, fm9_2 145, fm9s 185 ms). With 5 deg between the drop and QUAD4's envelope, QUAD4's tracker latches
"payload tilt > 60" before `/fleet/payload_dropped` arrives. This is exactly k1ob_2's log (carriers' requests 209.016,
QUAD4's ENVELOPE FAULT 209.0756, planner DROP 209.0756). In k1ob_2 the mux was 'partner' (the 5 Nov fix now covers that).
With the mux 'ours' the disarm reaches QUAD4's radio. Then `fleet_manager_node.py:393-402`: the mux is not
partner/cut, and a welded start publishes no `/drone_3/on_load` (`dissipative_node.py:822-823`, welded_on_load off). So
it is "EMERGENCY STOP for the whole fleet": every carrier is disarmed at ~1-2 m instead of the drop-and-land that the
locked `drop_on_loss` exception promises. After a take-back (QUAD4 ours, off the ring) the same race applies.

**Smallest fix (a gate change: Wesley's word, critic or waiver; tree frozen).** In graph_attach.py:877 pass
`**tracker_drop_options(context)` instead of `**tracker_land_options(context)`, so QUAD4's own drop rule requests the
drop and stands it down like the carriers. Its rule is already skipped while the mux is 'partner' (tracker_node.py:934).
An alternative is for every tracker to subscribe to `/fleet/drop` and set `_drop_requested_t` on any request. Without a
fix, the residual risk is on the drop-55 lines: the sheet's manual LAND at ~40 deg is the only margin.

## Checked and found sound (no finding)
1. **LAND defaults (real.yaml: descend_to_stall, rearm, carriers_down_alone).** The planner
   (`planner_node.py:1338-1379`) releases carriers only while the LAND waits for a departed drone, after a touchdown *by
   stall* (`_td_by_stall`; a max-drop touchdown releases none), with the ring down, the ring and every carrier
   <= 0.05 m/s for 0.5 s on fresh poses, and no take-back in hold/guard/guard_follow or with a failed guard
   (`dissipative_node.py:1338-1347`). The tracker (`tracker_node.py:1442-1466`) idles and asks only within -3..+15 cm of
   its own TAKEOFF-latched rest, still, with its reference at the floor. A carrier that close to the floor cannot hold
   the ring on a 1 m cable. The ask is one-shot. The manager (`fleet_manager_node.py:325-352`) needs this flight's
   release. A refused or lost disarm leaves an idle carrier that LANDED disarms; the wait always ends (15 s, then landed
   anyway). The latched flags are cleared at ARM and at LAND. QUAD4's subscription only acts if it is in `slot2drone`
   during a wait, and its rest is the floor on welded starts. Rearm: an arrival-stale pose holds the stall count; the
   descent is bounded by LAND_MAX_DROP. No path found that disarms a carrier holding the ring in the air, or leaves one
   armed.
2. **QUAD4 ring checks follow the mux.** The QoS matches the mux's latched publisher. `payload_dropped` is published
   before the take-back (`dissipative_node.py:1959-1963`), so the 'ours' after a drop-take-back arrives later
   (k1ob_2: +2-7 ms).
3. **Te-Jen fork.** The gate opens only on `{"running": true}`, which is sent only after the 2 s still step-out
   (`dissipative_node.py:2393-2440`). Held: his reference is the vehicle's pose, and our mux drops his stream (start_ours).
   An in-air refusal holds `air_hold_position` and retries at every 1 s release. On the mat it refuses as before.
   Console alarm logic OK.
4. **Skew/stamp logging** is inside the never-raising tick log. `detect_skew` is fully removed (grep). `ring_low` is off.
5. **Scripts.** Every ARGS key of all 19 Wed/Thu scripts is a declared `real_control_launch` argument
   (`profiles.declarations('real')`), and every HIS key is declared in `partner_mission.launch.py`. No param was
   silently dropped. Params-file keys map to PLANNER_OPTIONAL/TRACKER_DROP.

## Minor notes (not blocking)
- nro/nros: if his TAKEOFF is refused in the air, his phase stays WAIT_FOR_TAKEOFF. The attach pause then never starts,
  and the circle keeps running past a world-fixed hovering QUAD4. This is the same as our own step-out hold before his
  TAKEOFF (nr9o_2 flew it); LAND if it happens.
- Pre-existing, not new: a taken-back QUAD4 still airborne 15 s after the carriers' touchdown is disarmed by LANDED.
  The 5 Nov waits were 6.5-8.6 s, and the rearm's later touchdown adds margin.
