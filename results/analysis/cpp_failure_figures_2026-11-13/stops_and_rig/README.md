# Why his C++ planner stops: the moving ring, and the 29 Oct flights (13 Nov 2026, debugging figures for Te-Jen)

These are debugging figures, not results. They show what his planner and backend did, step by step, so he can tune them.
Part A uses open-loop real-time replays of the 12 Nov circling flight `nro`. Their data is in
`../../cpp_realtime_2026-11-13/full/runs/<run>/`. In a replay the virtual QUAD4 follows his published reference exactly,
so "QUAD4" means his reference. The carriers and the ring are the real 12 Nov mocap poses (`nro_poses.csv`, 20 Hz). The
replay clock is mapped onto the flight by fitting the ring track, as `full/dist.py` does. Every run fits at 64.18 s,
with a mean ring error of 1-2 mm. All distances are between centre lines, using `full/dist.py`'s geometry:
- QUAD4's hanging line runs straight down 1.134 m from its body (1.034 m cable plus the magnet and the object, with no
  swing).
- Each carrier's tether runs from its body (0.04 m below the centre) to its plate (0.225 m from the ring centre toward
  that carrier, at ring height).

"Capture" is the replay driver's test: QUAD4 within 0.12 m of the target in x-y and 0.15 m in z for 0.5 s. The settings
of all Part A runs are outer_octagon, swing 5 deg, tracking 0.06, 30 steps, refresh 0.2 and binary v2 (02:03:52). Only
the knobs named per figure differ. Times are replay seconds from the replay start, and authority is granted at +2.1 s in
every run.

Scripts: `common_a.py` (loader and geometry) and `parta.py` (figures 1-4), run with `python3 parta.py`. Part B's scripts
are listed below. Nothing outside this folder was changed.

## Part A: the moving ring. His planner stops in the ring's path

### fig1_r2_zero_window_topview.png: the sweep, seen from above (nro_mv_z29s20sp3_r2)
Settings: spline 3, z cap 2.9 m, splice 0.20 s. There are six top-view snapshots from +25.5 s to +32.6 s. Each one
shows:
- the ring (grey disc, plate radius 0.225 m) and the planner's 0.56 m ring outline (dotted);
- the ring's direction of motion (black arrow);
- the three carriers d0-d2 with their tethers drawn to the plates;
- QUAD4 (red dot) with its last 3 s, and his target (green X, at the ring centre).

At +27.5 s, QUAD4 stops 0.13 m from the target, just outside the 0.12 m capture radius, over the edge of the ring. From
then on the ring keeps circling away from it at 0.10-0.15 m/s on its ~0.6 m loop. QUAD4 never moves, so d1's tether
(orange) swings round and passes through QUAD4's position: 0.38, 0.29, 0.17, 0.04 and then 0.00 m at +32.6 s. The
heights overlap, so the top-view crossing is also a crossing in 3-D. QUAD4's line hangs from 2.58 m down to 1.45 m, and
d1's tether runs from 1.93 m down to the plate at 0.97 m. From +29 s the backend is in UNCERTIFIED_HARD_HOVER. What to
read: the stop is not a stop in a safe place. The planner stops wherever its last plan ended, even when that point is
in the ring's path.

### fig2_zero_window_timeseries.png: why it stops (r2, and the same run with 8 busy CPU loops, _load8)
The two columns are the same configuration, run alone and with 8 busy loops. The rows, from top to bottom, are:
1. QUAD4 and ring speed;
2. the horizontal distance from QUAD4 to the target, and the closest carrier tether;
3. commit end - now, from the backend's 1 Hz rows;
4. each replan's runtime against the splice lookahead (200 ms), coloured the same way as row 5;
5. every replan request by result.

Orange shading marks the backend's execution fallback (UNCERTIFIED_HARD_HOVER, reason INCUMBENT_REVALIDATION_UNSAFE).
Pink shading marks QUAD4 standing still.

How it fails in r2:
- Once QUAD4 is within ~0.5 m of the target (+25.3 s), the planner's candidates start failing
  (LOCAL_PLAN_FAILED:PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED).
- One complete moving-rendezvous plan is installed at +25.9 s.
- The next two candidates take 213 and 219 ms. That is longer than the 200 ms splice lookahead, so both are rejected as
  late.
- At +27.1 s the installed plan re-checks unsafe against the moved ring: 0.33 s to a conflict, then 0 s.
- From then on, every request ends INCUMBENT_REMAINING_UNSAFE_AFTER_LOCAL_PLAN_FAILED. These return in ~26 ms (median),
  far quicker than a real search.
- commit end - now sits at about 0 s, and QUAD4 holds still for 10.5 s (+27.3 to +37.8 s) while the tether closes to
  0.00 m.

With 8 busy loops (_load8), the approach runtimes are 200-300 ms. That makes 29 of 153 candidates late, and only 20
replans are accepted in the whole run. The last accepted plan ends short of the target, and QUAD4 has stopped by
+9.3 s while ~2 s of that plan are still committed. Then the same pattern follows: 82 incumbent-unsafe results at a
30 ms median, and hard hover in two blocks covering 19 s. QUAD4 is still for 29.6 s (+9.3 to +38.9 s), and the tether
sweeps through it twice (0.00 m at +16.3 s, 0.07 m at ~+29 s).

What to read:
- Near the ring, no new candidate passes the padded final check. The fast ~25-30 ms failures suggest that the start
  point itself is already inside the padding, but the data does not show which check fails.
- When the incumbent turns unsafe, his only response is to hold still. There is no move away from the swept area.
- CPU load makes the approach worse by turning candidates late. It does not cause the stop.

### fig3_z38_moving_capture.png: the contrast, a moving capture with no stop (nro_mv_z38s20sp3)
Settings: the same as r2, except the z cap is 3.8 m (spline 3, splice 0.20). The panels are the same as figures 1 and
2. QUAD4 comes straight in at up to 0.22 m/s, and the plan is re-installed every 0.2 s until +11.5 s. Near the ring the
candidates fail here too (LOCAL_PLAN_FAILED from +11.3 s). The difference is timing. The last installed plan still has
1.5-2.5 s committed when QUAD4 enters the capture radius, so it carries QUAD4 in at 0.07-0.09 m/s, about the ring's
speed. Capture comes at +13.3 s, 11.2 s after authority, with the closest tether at 0.42 m. Only after the capture
does the same INCUMBENT_REMAINING_UNSAFE / hard-hover pattern start (+14-22 s). What to read: success depends on the
last good plan reaching the capture radius before it runs out. The planner does not keep finding plans near the ring
in either run.

### fig4_p02_parked.png: parked beside the path, never captured (nro_p02)
Settings: splice 0.10 s, spline 4, no z cap, everything else the same. Up to +39 s, 175 of 431 candidates are rejected
as late. Runtimes are 97 ms median and 147 ms p95 against a 100 ms splice, and QUAD4 follows the ring round without
closing in. From +39.1 s to the end (+90 s), QUAD4 is parked at x 0.16-0.29 m, y 0.67-0.74 m, beside the ring's loop.
The same cycle repeats about every 15 s, once per loop of the ring:
1. The ring comes past, with QUAD4 0.43-0.5 m from the target.
2. Candidates fail, the incumbent re-checks unsafe, and the backend holds hard hover (+43-49, +57-64, +72-79 and +87 s
   onward).
3. The ring moves away, and one more ACCEPTED_PARTIAL plan is installed (commit about 7 s). Its committed_endpoint_distance_m
   is 0.60-0.68, and QUAD4 hardly moves on it (0.03 m/s at most).

The runtime row shows why so little gets in. In each gap between ring passes (+49-57, +64-72 and +79-87 s), 21-27 of
38-39 candidates are rejected as late, with a median runtime of 138-142 ms against the 100 ms splice. Before +39 s, the
installed candidates take 97 ms (median) and the late ones 115 ms. The octopus search has a 50 ms wall budget, but the
total replan time is about double that. Closest tether: 0.19 m at +76.2 s. What to read: with the shorter splice, most candidates arrive late. The plans that
do get installed are short partial moves that never close the last 0.4 m against a target that keeps moving.

## Part B: the 29 Oct real flights with his C++ planner and the carriers (ring still)
Data: `results/rig/2026-10-29/<run>/<run>_run/logs/run.csv` (mocap at 50 Hz; QUAD4 is d3) and `<run>.log`. From the log
come his online_join_planner lines and the backend's once-per-second status blocks: latest_status, replans
accepted/failed/late, the target and enabled. Time zero is the manifest's t0_epoch, about 0.1 s after his TAKEOFF
command. All three runs flew code 413ec0e plus uncommitted changes. Script `partb.py` (`python3 partb.py`) makes the
three figures and prints the numbers quoted here, which are also saved in `partb_numbers.txt`.

### fig5_sq3_handover_speed.png: the stationary hand-over is never granted (sq3)
Settings: typed floor 2.36 m, hand-over speed bound 0.08 m/s (his rig default), drift bound 0.15 m, dwell 0.5 s. His
code's own settle dwell is 0.30 s. The panels are:
1. QUAD4 speed from mocap: raw, plus a 0.3 s centred mean. His 16 'stationary handoff state settled' lines (green) and
   11 'left the prepare bounds' lines (red) are drawn at the speeds he logged. Green bands are the settled spells.
2. Drift from the latched climb-end position, with the 0.15 m bound.
3. z, with the floor.
4. A strip of the backend status.

His logged speeds sit on the mocap curve, so the velocity shown is the one his planner saw. In the hold, QUAD4's speed
has a median of 0.057 m/s and a p90 of 0.095 m/s. It is over 0.08 m/s 22 % of the time. Every settled spell ends
within 0.03-2.1 s. Every exit is on speed (0.080-0.092 m/s). Drift (at most 0.106 m) and acceleration (at most
0.39 m/s²) stay inside their bounds. What to read: 0.08 m/s is inside the real QUAD4's hover noise, so the hand-over
cannot hold. Behind that there is a second blocker that the registry row does not name. The backend was
WAITING_ABOVE_SEARCH_FLOOR for the whole hold (25 blocks, 0 replans, enabled false), because QUAD4 hovered at
2.14-2.20 m, 16-22 cm under the typed 2.36 m floor. Even with a looser speed bound, the C++ transit would not have
searched. The operator took QUAD4 back at 40.6 s.

### fig6_sq4_5_search_floor.png: settled, then waiting above the search floor (sq4_5)
Settings: the same as sq3, except climb speed 0.3 and hand-over bounds of 0.30 m and 0.12 m/s. The panels show:
1. z, with the 2.36 m floor, the ring height and the capture-target z sent to the backend, plus a zoom on the hold;
2. speed, with the 0.12 m/s bound and the settled/left markers;
3. the backend status strip.

The hand-over settled at 9.3 s, left once at 12.7 s (at 0.126 m/s) and settled again at 13.1 s for good. From 9.3 s to
33.3 s the backend reported WAITING_ABOVE_SEARCH_FLOOR. Replans accepted/failed/late were 0/0/0 for the whole flight.
QUAD4 held 2.30-2.37 m (median 2.330) and was above 2.36 m only 2.2 % of the hold. Its one crossing (27.2-27.7 s)
changed the status for a single block to RECOVERY_WAITING_EXISTING_TRACKING_ENVELOPE. No plan followed, and QUAD4 sank
back under. The capture-target z was 2.24-2.45 m (median 2.345), and 76 % of those targets were also under the floor.
The ring sat at 0.976 m, and the take-back came at 34.2 s. What to read: the climb ends ~3 cm under the floor, and the
backend waits for QUAD4 to rise above it. Nothing commands that rise, so the floor holds the planner off for good. The
floor needs to sit under the climb's end height (and the target's) with a margin larger than the hover noise, or the
climb has to end above the floor.

### fig7_sq2_5_transit.png: the C++ transit flies 8 s, then a brake whose restart is blocked (sq2_5)
Settings: his default high transit, with no floor typed. On the left is a top view: the path coloured by phase, the
carriers and the ring at capture, the plate-9 target and the violation point. On the right are three panels: z (with a
zoom on the floor), horizontal speed with the xy distance to the ring centre, and the backend's cumulative replans.

The sequence:
1. C++ takes authority at 23.9 s. In 8 s it closes from 1.82 to 0.37 m from the ring centre, at 3.16-3.24 m and up to
   0.44 m/s, with 27 accepted, 18 failed and 0 late replans.
2. At 31.9 s the backend logs a tracking-tube violation: a position error of 5.3 cm in x against a 5 cm bound, at
   0.13 m/s. It installs a SMOOTH_BRAKE fallback.
3. For 12 s the status is EXECUTION_FALLBACK_RESTART_BLOCKED_BELOW_MIN_SEARCH_Z, and the replans stay frozen.
4. At 43.7 s Python's capture-region check takes over (APPROACH_ABOVE_TARGET).
5. MATCH_VELOCITY runs for 12.3 s. The descent is aborted after 0.4 s ('loss of relative alignment'), then MATCH runs
   again until the take-back at 68.5 s.

The floor drawn at ~3.20 m is an estimate from the launch's formula at 413ec0e, which gives 3.197 m. The log does not
record the floor. QUAD4 was at 3.194 m at the violation and 3.159-3.195 m in the hold. What to read: the registry's
"C++ transit reached its capture region in 20 s" was 8 s of flight and 12 s of braked hover. Two of his thresholds sit
inside real hover noise here. The 5 cm execution tube tripped on a 5.3 cm error. The search floor sat ~0-4 cm above
the hover height, so the brake could not restart.

## Across all seven figures
- The planner has no move for "my plan is no longer safe". It holds still, whether the hazard is moving (figures 1, 2
  and 4) or the hold is blocked by a floor (figures 5-7).
- Several of his thresholds sit inside the real QUAD4's noise: the 0.08 m/s settle bound, the 5 cm tracking tube, and
  a search floor within a few cm of the hover height.
- Near a moving ring, candidates fail the padded final check (LOCAL_PLAN_FAILED:PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED),
  and a replan runtime longer than the splice lookahead makes them late. Splice 0.10 with runtimes of 100-160 ms is
  mostly late.
