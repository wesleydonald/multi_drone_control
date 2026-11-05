# Session state (the LAB chat reads this first, via `/lab`; keep it short and current)

Updated: Thu 5 Nov 2026, 18:38 AEDT. Branch **week8** at 9f20f8b, uncommitted work (commit only on Wesley's word). **The lab day of 5 Nov is over (Wesley ~11:05: "record everything").**

## RUNNING: the final-tree claim batch, started Thu 5 Nov 18:25 (Wesley ~18:00: "Start running any additional simulation results I need")
- `~/mdc_overnight/claim1107.sh` (pid in ~/mdc_overnight/claim1107.pid = 25999; log ~/mdc_overnight/claim1107.log): one sync + build
  of the tree as it was at 18:25 (9f20f8b + 509 dirty; gate --quick PASSED 18:24, 2200; 135 configs --check ok), then
  extra1105_nros (sq_nr_orbit_L100_1104_rigargs_pause: the square on the orbit, rig prep for Thu 12 Nov), then chunks 1-9
  (docs/claim_batch_2026-11-07.md; ~22 h, 2 slots x 12 cpus). **THE TREE IS FROZEN: edit nothing under src/, tools/, configs/
  (configs/rig/ too: the one-tree hash reads it) or simulation_assets/ until the last chunk has started.** Docs, results/,
  cad/ and the scratchpad are fine. Stop: kill the script's pid, then the running lab_batch (it removes its containers).
- After each chunk: registry rows, the one-tree hash check (manifest section 5), the void re-flies, score (section 10), feed, ticks.
- Done since 18:25: R3771 (the square on the orbit) PASS, Thursday's nros cleared; the X4 chunk running (R3772-R3776). The
  k1ob_2 jerk EXPLAINED (the bigger object's impact pulled two magnets off; RIG-1105-k1ob_2c, feed F051); the planner's late
  carrier poses recur (nr9m_4, sq8_2: 2 of 19 flights; decisions FINDING; QoS look after the freeze); no frozen bars (Wesley, rules
  + memory + sheets); Te-Jen's notes updated (send: Wesley); handin_plan ticked.
- **Also running (laptop) since 18:52:** X3b's timing runs R8b/R9b (`~/mdc_overnight/x3b_tuf.sh`, pid in x3b_tuf.pid, log
  x3b_tuf.log; solvers in ~/mdc_x3b_generated; ~2.5 h). Keep the laptop light: no agents or heavy analysis in each run's first ~3 min.
- Wesley ~18:50: funnel v2 smaller and shallower (built: cad/funnel/funnel.py --variant v2; 174 x 144 mm, 56 mm, 27 g; v1 kept; both in
  ~/Downloads/funnel/; by Thursday's 17 misses v1 catches 10, v2 8 (magnet into the mouth)); NO ring-mass change for the funnel;
  match_dwell explained (his gate's unbroken hold before the descent; his default 1.0, the rig typed 0.5).
- Waiting for Wesley: which funnel to print (v1/v2); match_dwell 1.0 on Thu (fly or skip).

## Thu 5 Nov evening (from 16:47, after Wesley's restart)
- **Verify of the 4.3 join cells (tracking check 0.20 m):** R3766 PASS; R3765 joined (the landing's abort = the known twin
  artefact: QUAD4's own 60 deg envelope in the post-touchdown unwind, scored apart; R3765c); R3767 (pickup, low route) stalled
  0.7 m short of the drop point (R3767c) -> Wesley ~17:40: this cell flies OVER the carriers (an exception in CLAUDE.md);
  claim1107_8b now lists ..._noxy_over.
- **Rig LAND defaults (real.yaml):** land_descend_to_stall + land_rearm_touchdown + carriers_down_alone (15 cm), built ~17:35 with
  launch tests and the re-arm's logging should-fixes; gate PASSED (2200); CLAUDE.md and Wednesday's sheet (section 4b) say so.
- **Running 17:44:** lab PC batch (task beqc36fpd, configs/batches/verify1105b.txt): the pickup cell over the carriers, and
  cda_{b,d}_k1s_rearm (the stronger carrier with the re-arm on: no false touchdown expected).
- **Done 17:53:** R3768 (pickup over the carriers) PASS: dropped, re-attached, object in the net 3 mm off centre; R3769/R3770 (the
  stronger carrier with the re-arm on) touch down at +0.00, no flip; with carriers_down_alone each disarmed alone 0.6 s after: H1-H5
  hold. Registered; feed F050. **The weekend batch is ready** (docs/claim_batch_2026-11-07.md); Wesley's code freeze is the trigger.

## Thu 5 Nov afternoon: the main chat distributes the work to agents (Wesley ~12:45); the lab PC for simulation
Updated 16:17. **Order of work (handin_plan):** code freeze Fri 6 Nov -> every simulation claim re-flies on that tree Sat 7 -
Sun 8 (docs/claim_batch_2026-11-07.md: 255 runs, ~22 h, scorers tools/*_metrics.py proven on the old runs) -> simulation freeze
Mon 9 -> rig S3 Wed 11 / S4 Thu 12 (sheets docs/rig/wed_2026-11-11.md, thu_2026-11-12.md; freeze Thu 12 11:00).
- **Decided today (decisions.md 5 Nov):** QUAD4's tracker ring checks follow its mux (on); his TAKEOFF held until our release + no
  DISARM on an in-air refusal (rig); no loss-check timestamp skip (removed; the stamp-gap log stays); land_descend_to_stall = the RIG
  default (the slow descent deleted); the landing combination carriers_down_alone + 15 cm band (built, off; its 10 twin runs
  R3755+ flying; if clean -> rig default); the fold-in cap 'level' kept OFF, 'weld' deleted; the weekend batch: T joins int_k_xy 0,
  the gap table re-flies, no more 1.15 kg cells; Te-Jen's tracking check 0.20 m (margin 0.05; critic waived) + the plain floor-join
  on the low route (the three 4.3 join cells stalled at 0.05: R3738/R3749/R3750); rig sheets: M2A ring, Nr claim on the square,
  K at 1.2 m with drop_height 0.28 (k1z), Nr on the orbit typed; the small object for pickups.
- **Lab PC:** carriers_down_alone R3755-R3764 running (task b0qnqcefp); then verify of the three join cells (rebuild; task
  b1kel1r17, configs/batches/verify1105_x11_joins.txt); if they weld -> the weekend batch is ready.
- **Funnel (Wesley's attach aid):** cad/funnel/funnel.py (CadQuery in ~/venvs/cad_pkgs, PYTHONPATH), v1 40 g, bolts M4 through the
  inner-anticlockwise + both outer holes, nut pockets, the zip-tie hole clear; files in ~/Downloads/funnel/ (funnel_in_ring_frame.step
  lands on the 60 deg plate of his ring Part Studio). Waiting for his look in Onshape.
- **Still Wesley's:** the claim bars to freeze (sheets N1-N3, K1-K3), match_dwell 1.0, the funnel's mass trade-off (a 40 g
  counterweight or load_mass 0.90), the code freeze commit (not asked).
- Findings: sq8_2's carrier poses ~0.6 s late (cause not found); the triangle's misses radial; the square's T lean on today's tree
  differs from 01:30 only with the integral on (R3740 = R3709 with it off).

## Thu 5 Nov, the lab day (registry RIG-1105-*, decisions FINDING/OPEN 5 Nov, feed F048, notes docs/rig/notes_for_tejen_2026-11-05.md)
- **Worked:** k1d (pickup + drop into the flying ring, first try); **sq8_2: the first real detach and re-attach** (square 11/2/5 + QUAD4 on 8; contact_dz -0.03; proven, folded in 14.4 s after the release; the T leaned 21-28 deg; then a false loss in LAND dropped the ring); the orbit pause on the rig (nr9o_2); the bigger object dropped into the MOVING ring (k1ob_2, in the net 4 cm off centre) then the ring jerked to 54+ deg (drop) and **QUAD4's idle tracker disarmed QUAD4 (fell ~2.5 m): BUG, OPEN (a)**.
- **Cause of the morning's lift bounces: a Motive recording** (delay 45-65 -> 75-135 ms); rule in CLAUDE.md and the sheet.
- Typed today (params files in configs/rig/, scripts in ~/rig_*.sh and results/rig/2026-11-05/scripts/): drop 55 deg (attach_m1_1m_m2a_drop55*.yaml), the loss check 15 cm (`_det15`), his contact_dz -0.03, the orbit with no pause for the drop (`_orbitdrop`), the square on 11/2/5 + 8 with reattach_plate 8.
- Failed / open: re-attach accuracy on the triangle (nr9o_2, nr9m: 3.5-10 cm off); his TAKEOFF before the release (nr1r); pickups after body 6 was re-enabled off-centre (k1o_2); the false loss check in LAND (sq8_2).
- Simulation today: R3710 VOID (config), R3711 PASS (the drop into the moving ring in the twin, in the net 4 mm off centre).
- Disk: 596 MB -> 22 GB free (gz_record replays, OneDrive archives, the 4 Nov zips deleted; Wesley's OK).

## 4 Nov, the lab day, in one paragraph (registry rows have the detail)
1.15 kg ring: sq6b dropped at 33 deg in QUAD4's descent; switched to the **0.86 kg M2A ring** (body 8; W 0.0365 X 0.0039 Y 0.2271 measured 08:44, L 1.0525 typed). sq6m: **the first real join**. Then 13 flights: the **release worked on all six** that reached it (tilt 2.9-5.2 deg; the take-back worked every time); **no re-attach finished** (nr1_2 on plate 1: caught within 4 mm, plate rose 18-19 mm, rejected on the 3 mm taut margin; nr9m_4/_5 on plate 9: contacts 24-59 mm off, declared by the 60 mm radius, lost); **no pickup drop** (k1m_2-4: the object on its 1 m cable swung 0.23-0.26 m from the centre, his arrival check 0.18 m held 0.5 s never passed; k1m_3: his reference sank to 2.01 m, LAND's take-back lifted the object with the ring: 30.4 deg, automatic drop). nr9, nr1_2, nr1_3 flew the 1.15 kg config on the M2A ring by mistake; nr1, nr1_3, nr9m_3 VOID. Every flight's command line is in `results/rig/2026-11-04/launches.csv`.

## The command blocks of 4 Nov (M2A ring), for Thursday
env `MDC_RIG_ACADOS_ROOT=$HOME/mdc_rig_generated_m2a_0p86`; `AM` = attach launch with `params_file:=$C/attach_m1_1m_m2a.yaml` (`_around` for the route); `geometry:=rig_m2a` in partner_args; K 0.06 (**too loose: see the OPEN line**); G, T3, K1 as in `docs/rig/wed_2026-11-04.md` + the loosened G (decisions.md 4 Nov ~10:00). Lines: sq6m (floor attach), nr9m (`partner_attached:=true detach_unload_s:=3.0`), nr1r (+ `reattach_plate:=1`, `plate:=1`, `_around` params), k1m (`mission_mode:=pickup_delivery`).

## Wrong advice given 4 Nov (do not repeat)
- `ros2 topic pub /tejen/magnet/command OFF` or `/drone_3/magnet OFF` does NOT release QUAD4's magnet during his pickup: our magnet_gate holds his OFF in TRANSIT_TO_DROP_POINT and ignores ours before our span (k1m_3, k1m_4 logs). There is no working operator drop yet (OPEN proposal 3).
- Read `launches.csv` and every log before advising; the state file must list every flight.

## Code changed 3-4 Nov (all uncommitted)
- `dissipative_node.py`: `release_route()` + `partner_release_around` / `_r` (default off); test `test_release_route.py`. R3633, R3634.
- His planner (our fork): the takeoff from the air (unswitched; decisions 4 Nov); test `src/tejen_mission/test/test_takeoff_from_air.py` (3). `contact_descent_hold` (3 Nov).
- Typed knobs: the co-motion set (graph_attach COMOTION_KNOBS); his launch `descent_abort_speed`, `descent_hold`, `descent_gross_xy`, `pickup_proof_drift/_speed/_loss` (test `test_pickup_proof_only_when_typed`). CHANGES_FROM_TEJEN.md has 4 Nov with notes for Tejen.
- Gate `--quick` PASSED ~11:20 (2096 tests; on the tree before the operator OFF, whose tests pass on their own).
- **Operator OFF (built 11:25, default off):** `magnet_operator_topic:=/drone_3/magnet_operator` on the attach line; then `ros2 topic pub -t 3 /drone_3/magnet_operator std_msgs/msg/String "{data: 'OFF'}"` drops whatever QUAD4 holds, in any phase, and holds every ON until an operator 'ON'. magnet_gate.py `operator()`; tests in test_magnet_gate.py (2) and test_real_mode_launches.py (1). Not desk-checked on the rig launch yet.

- **Drop on arrival (built 11:30, default off):** `drop_on_arrival:=true [drop_on_arrival_xy:=0.12]` in partner_args; our fork of his planner releases at the transit height once the magnet is within the radius of the drop point for 0.1 s. Tests test_drop_on_arrival.py (4), test_partner_mission_rig.py (1); his suite still 24 old failures, 1159 pass. Simulation **R3675 PASS** (registry): dropped at 2.37 m, in the net 3.5 cm from the centre, ring 5 cm / 3 deg; QUAD4 bounced to 2.61 m after the release (over the 2.6 m ceiling: Wesley's call, on the sheet); not starved (wall as R2899/R2900).
- **Thursday's sheet rewritten 11:33** for the M2A ring (`docs/rig/thu_2026-11-05.md`): calls (K 0.025 and drop on arrival marked Wesley's word), re-attach, pickup k1d, release x2, triangle and square baselines; new `configs/rig/traj_hold_m2a.yaml`. **Desk check PASSED 11:48-11:55** (scratchpad desk1104/sweep.sh, domain 79): all 10 lines up, no refusal, read-backs as typed; solvers for the layouts built into ~/mdc_rig_generated_m2a_0p86. Re-check only if the tree changes.
- Not built, needs Wesley: the square detach/re-attach in simulation needs a new world (carriers 0/3/6, QUAD4 welded on 9, M2A ring): a world SDF is his word. The take-back dropping a held object (sheet rule instead: operator OFF before LAND). Battery compensation in his controller (the twin has a pack model: testable).

- **Te-Jen's review (~14:00) and what followed:** his OCP caps throttle at 0.60; with the object QUAD4 sat at the cap (17 s in k1m_3) and sank under his reference, which held 2.43 m (my "reference sank" was wrong: corrected in registry RIG-1104-k1m_3c, feed F041, decisions CORRECTION). `tools/rig_to_run.py` now adds d3_partner_ref_*, dN_partner_thr, dN_radio_thr, dN_mux from the bag (tests pass; old run.csv files are not rebuilt unless rig_summary re-runs). Built, off unless typed: `mpc_throttle_max:=<0.60-0.95>` (tejen_mpc/throttle_bound.py, run time, no rebuild), `lift_full:=true` (up, then over). Tests: test_throttle_bound.py, test_lift_full.py, test_partner_mission_rig.py; his suites: 24 old failures + his controller's 2 old style failures. Simulation **R3676 PASS** (k1_5_lift_full, all three switches): left the pickup at 2.43 m (R3675 1.31 m), level after, nearest cable 0.25 m (0.21), in the net 2.4 cm from the centre, QUAD4 bounce 2.63 m. Thursday's K1 now types `lift_full:=true mpc_throttle_max:=0.8`; desk re-check of nr9m/nr1r/k1d passed 15:29 (after `ros2 daemon stop`: the daemon breaks param reads, `!rclpy.ok()`). Reply drafted: `docs/rig/notes_for_tejen_2026-11-04_reply.md`. Next for Te-Jen: the forecast churn (ring twist input), drop on arrival on a predicted landing point, the release height over the net (measure the net).
- **The square (Wesley: build and run):** the worlds already existed (27 Oct). **R3677 PASS** on today's tree: square -> T -> square, re-attach first try 25 s after the release; ring 21.6 deg peak 0.7 s after the release, the T ~7 deg, then 1 deg. Thursday's sheet section 6 (sq9), desk-checked 15:45, marked Wesley's call with a suggested stop rule for its first swing.
- **Battery compensation built** (default off, typed `mpc_battery_comp:=0.022`): his radio throttle + 0.022 per volt below 23.5 V (cap +0.10), after his partial-step copy-back so his OCP/UKF keep the model throttle; twin controller now remaps telemetry. Unit tests pass; Simulation A/B running on the laptop from 15:50: nr_p9_pack218 (21.8 V, no term) then nr_p9_pack218_bc (with it), task bs0f8u5pk; keep the laptop quiet at each start (~15:50, ~16:01). I edited his controller and ran tests during R3677's first minutes (rule broken; R3677 checked: no import error, not starved).
- Zips for Te-Jen made: ~/wesley-rig-2026-11-04.zip (22 MB), ~/wesley-rig-2026-11-04-bags.zip (272 MB).

## Wed 4 Nov evening, 22:20-23:30 (decisions.md has every line; feed F045, F046)
- **Rod in the plan: flipped on 22:20, back OFF 22:50; Wesley ~23:00: the plan stays MASSLESS everywhere.** My 22:50 physics was wrong (CORRECTION line): the magnet's weight hangs on the cable while the plan books it on the drone, on the rig too, so the T leans on both (a limitation). **The twin's links now match the hardware** (rod 0.002 + magnet 0.038 at the plate, airframe 0.510; 80 world files regenerated, all generators --check ok; snapshot scratchpad simulation_assets_pre_link_2026-11-04.tgz). Every twin run before 23:15 flew the old link. Rod-in-plan configs with the old 0.075 rod now fail --check (retired).
- Weld filter 0.1 s the sim default (stays). X2's later-runs bar 5 deg. OPEN 761 closed.
- **Built (default off/as before):** `cable_pull_up_max` (tracker bound, 0); `ring_low_land_m/_s` (planner; proposed 0.30 m / 3 s from the replay of every rig flight: fires only on h3tp since 7 Oct, on R3444 at 815.8 s). The clamp's runs 3-6 (rod-in-plan T's) CANCELLED; its question on the massless T is open (the analysis: no claim cell's rider is on it).
- **Reviewer, X2-full and X8: supported with corrections** (cards; F045).
- **Orbit: R3696 PASS** (the rig's partner args): the circle runs on after the release, his TAKEOFF stops it, re-weld first try 12.8 s after the release, tilt peak 2.1 deg. On Thursday's sheet as section 5 (nr9o, Wesley's call: `contact_rig_orbit:=true`), params `configs/rig/attach_m1_1m_m2a_orbit.yaml`; desk-checked 23:09. R3685 VOID (his C++ transit stalled; not the rig's args).
- Desk check of every Thursday line on the final tree passed 22:40-22:46 (the rig launch does not read the twin worlds).

## Running
- Nothing. Laptop quiet. Next work (Wesley decides the order): the OPEN 5 Nov items (a) QUAD4 disarm bug first; X3b R8b/R9b laptop timing on a quiet night; score R3686-R3693; the ring-low gate proposal (0.15 m / 5 s); twin re-checks on the new links.

## To do, in order (Wesley 11:15: "Yes do that now")
1. DONE registry rows (16), causes (k1m_3 drop, nr9m_5 lost proofs, the pickups' transit), feed F040, decisions + OPEN, CHANGES_FROM_TEJEN, tests. Gate running.
2. Wesley's static measurements, if he does them: plate 9 centred and at its edge (catch window), plate 1, L held taut, weights (ring as rigged, QUAD4 as flown, object), calipers (magnet face, plate), Motive body 24 vs 21-23, photos. Read them into the contact radius proposal.
3. Build behind switches (default off), then simulation: `drop_on_arrival`, an operator magnet OFF the gate passes, the take-back dropping a held object over the ring, battery compensation for his controller (our affine voltage term), the square detach and re-attach in simulation (it ends in a T: drop_gap_deg 200 needed; the T settles ~10 deg in simulation, X1 sqT).
4. Thursday's sheet: rewrite for the M2A ring with today's lines and what step 3 delivers.
5. The lab PC batch part 3: RUNNING (above). Then score and register X2-full R3586-R3591, X8 R3592-R3593 + the new runs (Q007 item 3: Wesley's 3 Nov call to run on the current tree overrides the hand-in plan's "X2-full and X8 wait for the T fix"; re-fly after the freeze what changes).
6. Unexplained, look at later: the ring's z integral crossing 70% of its 0.30 m bound (the warning prints at +0.21) on EVERY attach flight of 29 Oct and 4 Nov, with the 1.15 and the 0.86 kg settings alike: not the ring's mass (the weighing was dropped); suspect the carriers' thrust (their kT trims learn +5-20% over 35.2); his transit reference sinking 0.4 m in k1m_3; why plate 9's descents end 24-59 mm off and plate 1's 3-4 mm.

## Waiting for Wesley
(00000) 18:00 ALL MEASURED and on Thursday's sheet: W 0.0491, X 0.0, Y 0.2293, L 1.034 (new body 24), PC 0.038 (small object 70 mm, 22 mm, 60 g). Final desk check 17:55-17:59 PASSED with them (k1d planned 2.56/2.76 m). Thursday's sheet is ready.
(0000) 17:52 re-measured with the new body 24 (magnet on plate 9): **X -0.0000, Y 0.2293, W 0.0491**, ring rest 0.0237 (08:44: 0.0039/0.2271/0.0365/0.0253). L pending (Wesley holds QUAD4 up; watcher task btgz45adk). T1 (t1_four_markers detach) running as task bkwli1ciq: stop it after.
(000) ~17:25 measured: magnet 20 mm, plates 55 mm, net 5 cm below the ring's bottom, small object 54 x 22 mm (0.060 kg in the repo, to confirm), body 6 defined, **body 24 redefined: W/X/Y/L must be re-measured** (afternoon sheet 1), PC for the small object (afternoon sheet 2). Thursday's sheet has PT 0.022, PS 0.054 and <Wed pm> placeholders; once they come in: desk-check k1d again (the ceiling check uses the object geometry). Final desk check 17:23-17:26 of nr9m, nr1r, sq9, k1d (drop 0.48) PASSED on the final tree; gate --quick PASSED 17:20 (after fixing test_command_execution's namespace for the battery term).
(00) Answered ~16:10: pickup may go to ~2.8 m (decisions); the net ~ the ring's bottom, shallow; small object 54 mm across; try the square Thursday. BUILT: `drop_height`, `max_body_z` (his launch, typed only); sheet k1d types drop_height:=0.48 max_body_z:=2.8 (desk 16:42: legs 2.58/2.58/2.78); simulation: R3680 VOID (my config put the args in the description; a repeat of R3676); R3681 PASS (drop_height 0.48: carried 0.52 m over the ring, in the net 4 mm from the centre, QUAD4 peak 2.76 m). Afternoon measurement sheet: docs/rig/wed_2026-11-04_afternoon_1quad.md. Battery A/B R3678/R3679 registered. Lab PC batch lab_1104c DONE 16:35, all 39 PASS; scored (agent, checked) and registered R3586-R3593, R3636-R3674 (R3593 fetched by hand from the lab PC); feed F042, Q007 ANSWERED, thesis chat told 17:05. Scorers: scratchpad score1104/{x2,x8,s8}. Reviewer not yet run on X2-full/X8.
(0) Today's answers (11:58): contact radius: he asked for context (given; awaiting his call, K 0.025 on the sheet). Pickup: supervisor wants the smaller object and a drop 0.5-0.6 m above the ring (QUAD4 ~2.65-2.75 m: asked if the pickup line may go to ~2.8 m) and up-then-over (building). Square world: YES build and run. Battery comp: YES build (our copy of his code is ours to change, record every change). Note to Tejen drafted: docs/rig/notes_for_tejen_2026-11-04.md.
(1) The OPEN 4 Nov line: contact radius, drop_on_arrival, operator OFF, take-back drop. (2) The rod's flip and the dip. (3) The weld filter's default. (4) The clamp bound. (5) The X2 bar's value. (6) Disk: worktrees 1.9 GB, Downloads 3.1 GB, only on his word.

## How to work (Wesley)
- In the lab: work fast, give full, checked command blocks; read the log before every answer. Agents: at most 1 overnight, 2 otherwise. Critic waived for all (3 Nov). No bars frozen for simulation. Commits on his word only, never push.
- Lab batches: `tools/remote/lab_batch.py`, `setsid nohup`, scripts in `~/mdc_overnight/`; `--cpus` at most 12; chain on a PID; edit `src/` with an atomic copy-and-rename when a launch or sync may start (symlink install).
- Rig: Motive streams 6,8,11,12,13,14,24 (attach); 8,11,12,13,14,21,24 (release, square). The first launch after a layout change rebuilds the solver for 20-40 s.
