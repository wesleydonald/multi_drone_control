# QUAD4's tracker gets the carriers' drop rule (card, 6 Nov 2026; NOT BUILT: Wesley's word, critic or waiver)

Found by a read-only review of the code that flies on Wed 11 / Thu 12 Nov (docs/reports/reviews_2026-11-05/A_rig_code.md,
finding 1), checked by the lab chat in the code.

## The gap
- The attach graph gives QUAD4's tracker the LAND knobs only: `tracker_land_options(context)` at
  src/bringup/bringup/graph_attach.py:877, which keeps `carriers_down_alone` / `carriers_down_band_m` and drops
  `drop_on_loss`, `drop_tilt_deg`, `drop_samples`, `drop_glitch_step_deg` (real_mode.py:251-276). The tracker's own
  default is `drop_on_loss` False (tracker_node.py:1000), so QUAD4's tracker never asks for the ring drop.
- Its 60 deg ring-tilt envelope stands down only while its mux reads 'partner', after the planner has dropped the ring,
  or within DROP_CONFIRM_S of its own drop request (tracker_node.py:969-983). No tracker listens to another tracker's
  drop request.
- On the lines that type the 55 deg drop (nrs x3, nrs0, nrtf, nro, nros: `drop_tilt_deg: 55.0`), a fast ring upset
  with QUAD4 welded makes the carriers ask for the drop at 55 deg and stand down; the planner acts on a drop request in
  34-185 ms on the rig (k1m_3, k1ob_2, fm9_2, fm9s); a ring tilting through 55-60 deg at a few hundred deg/s (k1ob_2:
  11 -> 42.6 deg in 0.1 s) reaches 60 deg first. QUAD4's tracker then faults, its disarm reaches the radio (mux 'ours'),
  and the fleet manager calls an EMERGENCY STOP for every quad (fleet_manager_node.py:399-404): every quad disarmed at
  1-2 m instead of the ring dropped and the fleet landed (the locked drop_on_loss exception).
- When it applies: rare (a fast upset with QUAD4 on the ring), but the cost is four quads falling.

## The change (one variable)
graph_attach.py:877: `tracker_drop_options(context)` in place of `tracker_land_options(context)` for QUAD4's tracker,
so it carries the carriers' drop knobs as typed. Unchanged: its drop rule does not run while its mux reads 'partner'
(tracker_node.py:934, the 5 Nov fix), so his flight is untouched; welded (mux 'ours') it behaves as a carrier.

## Checks before the rig
1. A launch test: QUAD4's tracker parameters include `drop_on_loss` and `drop_tilt_deg` as typed (and none when untyped).
2. The unit tests of the drop rule and the off-ring stand-down (test_off_ring_checks.py) pass with QUAD4's params.
3. Twin, exploratory, once each: (a) a square detach and re-attach (as R3712) with a magnet pulled off a carrier after
   the re-weld (a fast tilt with QUAD4 welded): the ring is dropped and every quad lands, no EMERGENCY STOP; (b) the same
   run without the upset: nothing changes (no drop request from QUAD4's tracker in a normal flight).
4. The rig desk check (results/rig/2026-11-11/desk/check_all.sh) and a gate --quick.

## Falsifier
QUAD4's tracker asks for a drop in a flight where the carriers do not (a spurious drop), or (a) still ends in an
EMERGENCY STOP.

## When
After the claim batch's last chunk has started (its chunks record the laptop's diff at each start; the lab PC runs the
synced tree), before Wed 11. If it is not built: the sheets' manual LAND at ~40 deg is the only margin.

## Critic
(pending)
