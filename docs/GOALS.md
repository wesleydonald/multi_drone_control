# GOALS: Tejen integration (M1, M2), controllers, attach transient, speed

Owner: Wesley. Drafted 2026-09-26 (after T0015 / W-21:36). The loop works through this file in order,
re-reads it every iteration, and updates the **Status** lines with run ids. Anything under
"Needs Wesley" stops the loop for that item and gets asked, never guessed. The working rules apply
(exploratory vs claim, one variable per experiment, registry row per run, never commit, never push).

---

## G1 (primary): M1 — attached start, detach, pickup, drop, **rejoin**, land

Scenario: four carriers on plates 1/3/5/9 lift the 0.86 kg ring and orbit; drone 3 (plate 3) detaches
mid-orbit, steps out and climbs, Tejen's mission picks the steel ball up, drops it into the net on the
moving ring, hands back; our tracker descends, magnet weld, OCP fold-in to n=4, LAND. Event-driven timing,
no dead time (config `configs/experiments/partner_attached_orbit.yaml`).

Done means (claim: 2/2 headless Gazebo runs, then Wesley watches one in the GUI):
- no envelope fault, no abort; every stage reached in order; LAND with all four disarmed;
- ball ends in the net (in or on the net footprint) at LAND;
- detach peak ring tilt <= 8 deg; orbit ring tilt mean <= 3 deg;
- **rejoin weld: peak ring tilt <= 5 deg, carriers' dip <= 0.03 m, ring tilt back under 2 deg within 3 s**
  (baseline R0646: 10.4 deg peak, 0.08 m dip, 7.5 deg mean 5-6 s after the weld);
- sim time TAKEOFF -> landed <= 140 s (R0645: 137 s), wall time as low as G5 allows.

G1b (variant, after G1 passes): **the object has its real mass.** Today the ball
(`simulation_assets/tejen/payload_model.sdf`) weighs 0.01 kg, i.e. nothing. With the real mass: drone 3
carries it on the tether during pickup/transit, then the ring carries it after the drop. Done means, in
addition to G1: no fault while drone 3 carries it; after the drop the ring height returns to within
0.03 m of the target within 5 s (height integral / kt_trim absorb the step) and ring tilt <= 3 deg with
the off-centre mass; clean LAND with the object in the net. Mass ~0.1 kg (Wesley).

Status (2026-09-27): CLAIM SUPPORTED by the reviewer: R0685 + R0686 meet every G1 bar (rejoin 1.7/1.6 deg, dip
0.017/0.018, ball in net, 115/133 s) with the _uu rod model (sim workaround for the gz weld spin-copy) + four opt-in
flags (card docs/experiments/2026-09-27_m1_rejoin.md). Left: Wesley's GUI watch; his word on adopting the rod model
and flipping the flags to defaults.

## G2 (primary): M2 — Tejen's four-drone ground join, hand-over to our controller, lift, hover, land

Driver: `M2_LIVE_KI=10.0 python3 tools/sim_test/drive_m2_handover.py LOGDIR false 10` (GUI: `true 20`).

Done means (claim: 2/2 headless, plus one GUI run for Wesley):
- his join 4/4 with the transit token, no stall; our takeover with no envelope fault;
- ring to 0.60 +- 0.05 m and holds; ring tilt <= 5 deg (was 3, Wesley 2026-09-28) through sweep, lift, hover and LAND;
- drone body tilt <= 25 deg (the 45 deg rod pull needs ~14-15);
- LAND -> /fleet/landed, all four disarmed.

Status (2026-09-28): claim SUPPORTED on the gyro default (T0030/T0031, 5 deg bar; reviewer SUPPORTED with wording); GUI watch owed. Earlier (2026-09-27): claim WEAKENED by T0020 (ring rock 7.07 at ki 5; now 2/3) -- was SUPPORTED on T0018 + T0019 (ring tilt 2.38 / 1.83,
ring 0.601 / 0.600, 0 faults, landed; card docs/experiments/2026-09-27_m2_handover.md). ki 10 arm 1/2 (T0017 ring
rock 4.74). Left: Wesley's GUI watch (`python3 tools/sim_test/drive_m2_handover.py LOGDIR true 20`).

## G3: the attach (rejoin) transient

Symptom (Wesley): the newcomer comes in gently but the weld "rocks the whole system in an unnatural way".
Numbers: R0625 8, R0633 16, R0640 25, R0642 35 (one sample), R0645 23.5, R0646 10.4 deg peak after
welding at rest.

Hypotheses, each to be tested on the smallest bench that shows it (G5):
- H1 **sim flight controller cannot hold a steady torque** (P-only rate loop, ~3e-4 N m per deg/s):
  any rod load step through a lever rocks the drones. Fixed for M2 by the I-term (S0001-S0003, T0015).
  First test: the same rejoin with rate_ki 10.
- H2 **rod joints are undamped**: DART ignores `<axis><damping>` on ball joints (S0005-S0008), so the
  0.1 damping in our models is a no-op and the ring + rods + ball joints ring freely after the impulse.
  Test: damped universal joints (S0009/S0010 settle a swing in 0.6 s).
- H3 **weld impulse**: the DetachableJoint makes a rigid constraint in one step; any relative velocity or
  position error becomes an impulse (the at-rest threshold helped, 23.5 -> 10.4).
- H4 **reference step at the OCP resize**: n=3 -> 4 changes every incumbent's tension/position reference
  at once (10 s slew exists; check the tension FF and the gate/tautness jumps in the tracker log).
- H5 **tracker-side switches at the weld**: mux authority, payload_resting/cable-FF gate, kt_trim state.
- H6 **pivot lever** (0.05 m below the body): cleared for our newcomer by moving it to the centre (R0552);
  confirm nothing still hangs below the CoM.
Output: a ranked cause with evidence, a fix, and G1's rejoin criterion met.

## G4: controller review and improvement

Review, with the numbers to back each change (one variable per experiment; critic for any new flight-loop
law or estimator, reviewer on the final tables):
- tracker (controller_quad_load MPC): cable FF gating, kt_trim, attitude FF, behaviour under slow ticks;
- planner OCP (controller_load_mpc) + dissipative node: attach/detach resize, reference slews, creep
  (incl. airborne_start), z_ki, landing; the OCP's slow ticks (up to 1.2 s under load);
- elrs_mux / HandoverPolicy; fleet manager gates;
- interplay with Tejen's MPC and planner (hand-over instants, magnet commands);
- sim plant realism: rate-loop I-term (**decided: default**, see Order step 2), tether joints (bench first).

## G5: faster development and testing

- **Smaller benches** (minutes, not 15-25 min):
  - stage-1/2 single-drone tests (`tools/sim_test/stage1_torque_test.py`: torque, tether swing) exist;
  - M2 hand-over bench: world starting with his four drones already attached at his hold pose
    (skips his 15 min join);
  - weld bench for G3: three carriers hovering with the ring, the newcomer at rest beside the plate
    (`three_attach_rod_near.sdf` / `three_attach_noarm_near.sdf` exist), trigger the weld;
  - M1 without the ball mission: detach -> step-out -> rejoin only.
- Headless by default; GUI only to watch. Late controllers in M2 (done).
- Profile per scenario (CPU by node) and cut real waste: Tejen's per-drone nodes dominate M2 (planners
  163 %, MPCs 128 %, pendulum 92 %, observers 90 %, capture 79 %); rate changes there are his code and
  need a check that his join still passes. **Never throttle /clock in his world** (breaks his velocity
  estimates, W-20:56).
- Profile of M1 (R0704, 2026-09-27): CPU-bound at ~1111 % of 1200 % -- trackers 253, ros_gz bridges 110, sim
  Betaflight bridges 98, mocap emulators 84, runner 67, magnet managers 55, his nodes ~180, gz 51, desktop browser ~65.
  Options (not taken; lab workstation Wed): close the browser during headless runs; halve the 500 Hz model pose
  publishers (test copies + quality check, ~10-15 % RTF).
- Driver robustness: retries instead of crashes (done for rate_ki), resume mode (done).
- **Automatic metrics in every run's metrics.json** (so G1-G3 are judged from the registry, not by eye):
  per event (detach, release, drop, weld, fold-in, lift, LAND): peak ring tilt, peak drone tilt,
  carriers' height dip, time to settle under 2 deg, ring height error; M2: join time, hand-over-to-lift
  time, hover height/tilt, landed flag; plus wall time and mean RTF. One script used by both run_experiment.py
  and drive_m2_handover.py, with a unit test on a recorded log.
- Optional later: Docker image for a bigger machine / parallel headless runs. **Wesley 2026-09-27: no Docker for
  now; he will look into using his lab computer remotely** (Tailscale/VPN + SSH for headless runs, NoMachine or
  Sunshine/Moonlight for the GUI). Supervisor approved remote use; PARKED until Wesley is in the lab Wed 2026-09-30
  (Linux; he brings lscpu + the Tailscale name).

## G6: version control (DONE 2026-09-26: branch `integration`, commit d8aea49)

- New branch from `week1-ready-for-testing` (e.g. `tejen-integration`) holding everything since the
  Tejen integration; week1 stays as it is. Wesley commits.
- **Decided (Wesley 2026-09-26): source and worlds only** (src/, simulation_assets/); tools/, docs/,
  configs/ stay untracked as today.
- Keep out: `simulation_assets/tejen/logs/` (94 MB of his old logs, add to .gitignore), results, bags.
- The loop prepares the branch contents and the exact commands; Wesley runs the commit.

---

## G7 (NEW PRIORITY, Wesley 2026-09-28): work up to the real-world M1 and M2

Goal: a ladder of intermediate tests from where the rig is today to the full M1 and the full M2 on real
hardware, each rung small enough to fly in one lab session, each with a sim rehearsal that mirrors it, and
each gated on the one before. Lab access: weekdays only; next visit Wed 2026-09-30.

Every rung gets: what it proves; prerequisites (code fixes, hardware, Wesley's word); the sim rehearsal
(config that runs the SAME real launch files where possible, or the closest sim config); the desk check
(props off, fake_mocap or real mocap, nothing armed); the exact rig command sheet; the pass bar and the
abort criteria; the logs to read afterwards; the time it takes.

Draft ladder (the planning pass below checks every rung against the code and the rig history and
rewrites it):
- R0 desk: full stack bring-up with the real launches, parameter read-back, mux switching, magnet
  channel, abort path, Tejen's IRL stack talking to ours (handoff topics), props off.
- R1 one drone, our tracker, free hover and a small box; kT 24 check, kt_trim in shadow.
- R2 magnet hardware: drone 3's magnet on a fixed plate (weld and release by ELRS aux), no load.
- R3 carry hover with three drones (creep path), LAND.  R4 the same with four on plates 1/3/5/9.
- R5 slow orbit (0.125 m/s) with four.  R6 detach 4 -> 3 in hover, departed drone holds, LAND.
- R7 detach during the orbit.  R8 rejoin in hover (our approach, magnet weld, fold-in).  R9 rejoin
  during the orbit.
- R10 Tejen's side alone on the rig: his MPC flies drone 3, pickup and drop on a static target, then
  hand-back to our tracker through the mux.  R11 full M1.
- M2 track: M2a his single-drone ground join on the rig; M2b our takeover of four drones placed attached
  on the ground (the rig twin of the M2 bench): creep, lift, hover, LAND; M2c his four-drone join plus
  hand-over = full M2.
Known prerequisites to check: the yaw-datum reset on every OCP resize (a rotated ring on the rig would be
pulled toward yaw 0; see the control-math page, "Found while checking" 1); mocap velocity differenced over
packet arrival time (`motion_capture_publisher_node.py:239-265`); `real_attach_launch.py` manages only n
drones (the newcomer stays armed at LAND); weld detection on the rig (no Gazebo magnet manager);
rig layout plates 1/3/5/9 needs Wesley's word (decisions.md OPEN); rig checklist
`docs/experimentation/real_attach_gap.md`; 6S constants done 2026-09-24; Motive rigid-body origin at the
ring centre.

Status (2026-09-28): planning pass DONE -> docs/g7_real_world_ladder.md (rungs R0a..R11, M2a..M2d, prerequisites P1-P17,
Wednesday readiness, questions in its section 6). Loop checks applied: the mocap receive-loop bug is FIXED (ed7084d: a
malformed or undecodable packet ended the loop -> stale poses -> fleet disarm; offline test passes); R4v dropped (M1 never
used the velocity loop: zero phase switches in R0696). Rig fact: the ring has never been lifted on the rig; only free hover
passed (09-16, ~10-18 cm low, kT maybe ~22). Next: the Wednesday items (command sheet, rehearsal configs, preflight and
fake_mocap additions, cards for R3b/R4), then P-list items in order.

---

## Order

Readiness review (2026-09-29, Wesley: supervisors happy with sim; priority = the rig): docs/g7_readiness_review_2026-09-29.md
(+ _corrections, _gaps; page shared separately). Headline: the ring has never been lifted on the
rig and no tether has carried load; THROTTLE HEADROOM is the first question (rig free hover u 0.44-0.48, a 45 deg carry needs
~1.3-1.5x, tracker cap 0.6): weigh the airframes and compute per-drone u before any tethered flight. New intermediate rungs
(R0b', R3b0, R3e, R6a0, R8a0, R0c, R10a0, M2d0), Wednesday target R0b/R0b'/R2/R1/R3a, 16 decisions for Wesley, stale sheet items.
Prep done 2026-09-29: clean_slate --rig (spares T1/T4 and their DDS memory) + sheet reset fixed (4a5ecdc), repeated CLI ESTOP
(4a596f8), magnet latch + rosout bagged, R2 sequenced; tools/headroom.py (reproduces R0726 0.179 vs 0.181; planner tensions)
+ sheet step (4ef1fa7); requests for Tejen drafted (notes_for_tejen.md top). SIL R0a' at the rig thrust gain (thrust_c 21.8 = 9.81/0.45): 0.64 kg airframes pin all three trackers at the 0.6 cap for
the whole carry, ring sags to 0.45 m and tilts 16 deg (R0758); 1.0 kg holds 0.599 m at u 0.595 with ~0.005 margin (R0759);
tools/headroom.py predicted 0.68 / 0.59. => the airframe weights decide whether a 3-drone carry is flyable at all under the
cap: weigh them first; the cap question (raise it, Wesley's word) is now the likely gate for R3.
R0a desk rehearsal PASS (D0003): Wednesday lines, read-back, pre-TAKEOFF gate, clean_slate --rig (now spares fleet_viz); preflight
all-None radio read-back now FAILs; sheet notes added.
Docs aligned 2026-09-29: R3a bars (z < 0.15, tilt < 25, coupling, cap LAND rule) in sheet/ladder/card (5553a1a); CURRENT_STATE §9
hardware facts + open problem 6 closed; real_attach_gap.md marked superseded; stale sheet notes fixed.
preflight --detach (FAIL on reconfig_mode network / detach_magnet false; desk D0004) + sheet (1943776).
tools/rig_to_run.py: a rig flight (tracker logs + bag + T2 console, epoch time) -> run dir that metrics/plot_run read (2 tests;
09-16 16:36 converted: ring z 0.051 = never lifted); sheet: MDC_RUN_DIR per flight + scoring recipe.
Sheet: proposed SAFETY AND RECOVERY block (Wesley to confirm at the lab), per-flight pack/code record, kick_events on the flight folder.
SIL decision 3 (1.0 kg airframes, rig gain): even four-ring R0762 all drones 0.557, ring 0.599 m, tilt 1.2/4.0 deg (carries with
margin); plates 1/3/5/9 R0761 the plate-9 drone pinned at 0.600, ring tilted 7.2/20.1 deg. => R3e (even four) is the first-lift
fallback; the M1 layout needs more headroom (heavier airframes or a higher cap) before R4.
Sheet: conditional R3e rung + R4 headroom warning (9662004). NEXT SESSION (Gazebo budget resets): 1 run of
configs/experiments/ocp_hover_ground_creep_n4.yaml (even four-ring floor-start twin for R3e; R0468 was on the old plant);
then the M1 detach-upset watch (R0757) if runs remain.
Open for Wesley: the 16 review decisions (stop
value, cap, R3e fallback, attach_z, DDS domain, ...).
Loop status (2026-09-29 ~02:20): M2 JOIN STALL ROOT-CAUSED AND FIXED (37b3b5d, his fork, authority.md 2026-09-29): the cached
C++ window was padded with a moving terminal state, so a backend dropout longer than 2 s froze the position at 0.22 m/s and the
recovery check rejected every fresh reference forever. T0034: a 3.6 s dropout recovered, join 4/4, and abort-latch arm B PASS
(ESTOP before the hand-over: 4 muxes latched, 0 armed/above-idle forwarded, his streams armed, all four on the floor). The
latch card is complete (C, E, B). Also: 81239fd M2 /ours feedback remap contract test. T0035 (full M2 with the fix, 10 s hover
by mistake; claim runs used 20): join 4/4, landed, but lift tilt 10.25 > 5 from a carrier kick on drone 2 (mocap-rate glitch,
M-kicks mechanism, 10.3 kicks/100 drone-s vs 8.5 in T0031): M2 on the gyro default is 2/3 on the tilt bar; the kicks (parked
G4, question 10) now cost an M2 run. Headless runs this session: 6 of 10 (Wesley raised the limit to 10, 2026-09-29).
P9 bag reader ready for Wednesday (tools/r0b_mocap_report.py, in the R0b sheet).
Q10/Q11 ANSWERED. Q11 BUILT: pre-TAKEOFF disarm grounds our fleet + drone 3 on the manager's single command path (critic pass 2
READY; SIL R0753 clean). Verified 2026-09-29 (limit raised to 10 by Wesley; runs 7-9): D0001 desk forced disarm PASS; R0754 canonical M1 (drone 3 via
the manager, peaks 2.98/2.08/2.61) but his rejoin descent aborted 6x on 'loss of relative alignment' -> 146 s > 140 s G1 bar
(not attributed: in flight); R0755 creep hover 0.661 = R0557 (same config); R0756 attach demo weld 38.9 s, peak 3.61 (R0723
3.39). The approach MPC path has no current config (enable_approach_mpc false): unit + launch tests only.
Open: his M1 rejoin descent retries (R0706 1, R0752 1, R0754 6): watch; if it repeats, look at his alignment gate vs the
orbiting ring. R0757 (repeat): retries 1 / 105 s (R0754's 6 were chance) but a one-off DETACH upset: drone 0's body rates
swung +-1.4 rad/s 0.8 s after the release with its reference steady, it dropped 28 cm, ring 16.5 deg, recovered, landed. Not
attributed to the arming change; contact ruled out (R0757a: no rod collisions in the sim, drone 3 0.6-0.9 m away); kick family suspected, unconfirmed: watch the next M1 runs.
Headless runs this session: 10 of 10 (stop).
Loop status (2026-09-29 ~01:30): abort-latch Gazebo arms done where they can be. C with the rig's DISARM button
(R0751): drone 3 under his MPC and the carriers on the ground 0.5 s after DISARM, his stream still armed in the bag
(latch holds it). E (R0752 vs R0724): canonical M1 unchanged, latch inert. B (M2 ESTOP before hand-over): VOID twice
(T0032, T0033), his join stalls in drone 0's transit capture before our stack runs (his known stall, now 4/11; note 11
for Tejen); stopped by the time box. Committed on integration: 3b7a6a8 Q7 rig defaults, ced228b x0_rate_source (off by
default), 0aef5c7 abort latch + ARM gate + fault scoping + DISARM kill switch. Runner has a DISARM event. Gap fixed:
clean_slate.sh now kills m2d_commissioning_observer (it outlived a stopped M2 driver at 33 % CPU).
R0749 re-read (R0749c): not the n4 creep; all four trackers pose-timed-out between ARM and TAKEOFF and the manager
still took TAKEOFF (rig hazard if only some drop: one-sided lift). Card 2026-09-29_pre_takeoff_disarm_gate.md v2 (critic NOT READY, applied); build parked on question 11.
Next: question 11 (gate) on Wesley's word;
lab Wed 30 Sep (R0b bag for P9).
Loop status (2026-09-28 late night 2): answers Q6-Q10 + hardware facts + kill switch recorded. Committed: magnet path 6b8e01c, floor-start M1 b082bb3 (R0733 passes every G1 bar from the floor). Kick oracle: B0 R0734 / B1 R0735 -> gyro-fed x0 rate cuts carrier kicks 0.26x (fingerprint rule mixed), code kept pending Wesley. Built + reviewed, UNCOMMITTED: Q7 rig-mode defaults, abort latch + ARM gate + fault scoping + ESTOP event (GO for Gazebo arms C/E/B), operator DISARM latches the muxes (main.py). Next: fly abort arms C (ESTOP -> switch to DISARM event), E, B; then commit Q7 + abort latch (+ x0_rate if kept). Meeting brief page published for 29 Sep.


1. G6 branch (so the working state is safe before bigger changes).
2. **Sim rate-loop I-term as the default** (Wesley 2026-09-26): rate_ki 10 with anti-windup in every sim
   bridge (payload_betaflight_comm, tejen_betaflight_communication, betaflight_communication,
   angle_betaflight_communication); check his M2 join with it on from the start (the stalls were the
   simultaneous transits, now tokened); re-fly the baselines once each (hover, circle, detach, attach, M1, M2)
   and record the shift.
3. G5 benches needed by G3 and G2 (M2 hand-over bench, weld bench).
4. G3 on the weld bench (H1 is covered by step 2; H2 universal joints in test copies first).
5. G1 re-fly with the G3 fix; claim runs.
6. G2 second headless run + reviewer.
7. G4 review items as they come up; G5 speed items throughout.
8. (2026-09-28) Gyro default on every sim bridge: finish the flip, reviewer, re-fly each baseline once, M2
   claim 2/2 on fresh runs (5 deg bar), reviewer, Wesley's GUI watch.
9. **G7 real-world ladder (new priority):** planning pass now (read-only, parallel to 8): verify and rewrite
   the ladder, list the prerequisite fixes, then build the sim rehearsal configs and desk checks rung by rung,
   R0-R4 ready for the Wed 30 Sep lab visit. Prerequisite code fixes go through the usual card/critic rules.

## Concrete plan (planning pass 2026-09-26)

These rules apply to every step:
- Run ids are the next free ones in `results/registry.csv`. Step 2 uses R0647–R0650 and T0016. Later steps take the next R or T id.
- Every run gets one registry row, is exploratory until its claim, and changes one variable.
- Headless Gazebo runs: no cap (Budget section); a `/report` at each milestone.
- The Python install is a symlink/egg-link, so edits are live without a build. Never edit a bridge or node while any stack is up.
- Any change to a default, world, gate or geometry is made in reversible form: an opt-in arg, an env var or a test copy. A question for Wesley goes with it.
- Code that loses is deleted the same session.
- `rate_ki` changes at step 2, so every Gazebo comparison after step 2 uses a baseline flown with the same `rate_ki`.

### Step 1: G6 branch prep

**Options**
- **A (chosen).** Wesley creates `tejen-integration` and commits `.gitignore src simulation_assets`. Cost: 4 commands.
- **B (always done as well).** The loop makes a tarball and a patch. This is the only backup of `tools/ docs/ configs/ CURRENT_STATE.md authority.md`, because all of them are gitignored (`.gitignore` l.28–52).
- **C (out).** The loop runs stash, worktree or commit. That breaks the no-commit rule.

**Loop actions**
1. Append `simulation_assets/tejen/logs/` to `.gitignore`. It is redundant with `logs/` at l.25, but G6 names it.
2. Run `git add -n .gitignore src simulation_assets | grep -E 'logs/|\.bag|\.db3|\.mcap'`. It must print nothing. Today it lists 442 paths, the largest being a 2.8 MB STL, which is kept.
3. Make the backup:
   `mkdir -p results/backups && tar czf results/backups/pre_loop_2026-09-26.tgz --exclude=simulation_assets/tejen/logs --exclude=build --exclude=.pytest_cache src simulation_assets tools docs configs CURRENT_STATE.md authority.md .gitignore tests.txt results/registry.csv`
4. Write Wesley's commands into G6:
   `git switch -c tejen-integration && git add .gitignore src simulation_assets && git status --short | grep -v '^[AM] '`. Expect only ` M tests.txt`. Then `git commit -m "Tejen integration: tejen_* fork, M1/M2 hand-over, partner worlds, sim rate-loop I-term, attach/detach OCP resize"`.
   Do not switch to `main` or `term3` afterwards; they hold old copies of paths that are untracked here. The upstream is `origin/week1-ready-for-testing`.

**Exit:** the tarball exists and the dry run is clean. Step 1 is then done; Wesley's commit does not block the loop.

### Step 2: metrics first, then rate_ki 10 as the sim default, then one re-fly per baseline

**Options**
- **2a, metrics before any re-fly (chosen).** About 2 h of code in `tools/` and 0 runs. It re-scores R0640–R0646 for free. Without it, the re-flies cannot say which stage moved: R0646's WELD is NaN, REWELD is not in `EVENT_PRIORITY`, and there are no ball pose or sim stamps for his phases.
- **2b, rate_ki: node parameter default (chosen).** Set the default to `float(os.environ.get('SIM_RATE_KI','10.0'))` in all four bridges. `SIM_RATE_KI=0` gives a P-only arm and is also the one-line revert.
- **Rejected for rate_ki:** changing the `RatePid` class default (breaks `test_defaults_are_the_old_p_only_loop`, and the m2d env still forces 0), and per-launch args (Tejen's launches would stay P-only).
- **Re-fly order:** the floor start goes first. It is the only run that tests the one new risk, wind-up on the floor.

**2a Metrics (one script: `tools/metrics.py` plus `tools/experiment/runner_node.py`)**

Runner (`runner_node.py`):
- Add a `wall` column to `run.csv`, and `wall_s` and `mean_rtf` to `manifest.json`.
- Log `PHASE:<x>` from `/join_planner/phase`. Use String, RELIABLE + TRANSIENT_LOCAL, depth 10, with the QoS defined inline (no `tejen_mission` import).
- Log `HANDOFF` on true edges only of `/join_planner/handoff_ready`.
- Log `PICKUP` and `DROP` on the edges of `/tejen/object_attached`.
- Log `FOLD_IN`, `PARTNER_RELEASE` and `PARTNER_HANDOFF` from `/rosout`, filtered to `dissipative_controller`.
- Stamp all of these at the runner's sim_t on receipt. ROS log stamps are wall time and are not used.
- Log `ball_x/y/z` from `/model/payload_model/pose` index 1. Raise an error if the start pose is not (1.2, −1.4, 0.08) ± 0.02.
- Add config field `cmd_ready_topic` (default `/drone_{i}/ELRSCommand`).
- Add named events `HANDOFF`, `WAIT_PARTNER_RELEASE` and `HANGER_RELEASE` to the closed `EVENT_KINDS` list in `tools/experiment/config.py`.

Metrics (`tools/metrics.py`):
- Add `REWELD` to `EVENT_PRIORITY` (l.606).
- Add `event_window_metrics(t, data, t_event)`. It returns:
  - peak ring tilt and peak drone tilt;
  - carrier dip, actual and reference separately (the event drone is excluded);
  - time to stay under 2° for 1 s, reported as censored if the window ends first;
  - ring z error over the last 1 s.
- The window runs to the next disturbing event (DETACH, WELD/REWELD, FOLD_IN, a PHASE drop, LAND) or +15 s. RELEASED, WAIT_* and LIFTED do not end a window.
- Add `m1_stage_metrics()`:
  - drone 3 is masked from RELEASED to REWELD;
  - ball in the ring frame is Rᵀ(p_ball − p_ring). `in_net` means ρ ≤ 0.20 and z_rel in [−0.05, 0.12]. `on_footprint` means ρ ≤ 0.28. At LANDED only ρ counts;
  - reattach retries are counted only before the first HANDOFF;
  - the phase count is cross-checked against `grep "Planner phase:" extra_launch_1.log`.
- Add `summarise_m2(run_dir)`, whose CLI writes `metrics.json`. It reads the nested `logs/<name>/log.csv`, the dissipative CSV and the `runtime.log` path taken from `runner.log`. It reports join 4/4, fallback and TRACKING_TUBE_VIOLATION counts, envelope faults, max ref age, `load_z` from (first `z_tgt` == target) + 5 s until `land==1`, `tilt_deg` per segment up to touchdown at `load_z` ≤ 0.105, drone tilt ≤ 25°, and landed.

Tests (`tools/test/test_metrics.py`, `tools/test/test_experiment_config.py`):
- synthetic step and dip cases;
- a trimmed R0646 fixture, expected to give reweld peak 10.43, detach peak 5.41, carrier dip 0.063 (d1), settle 9.39 s;
- the T0015 M2 files, expected to give ring tilt peak 1.78° and `load_z` 0.603–0.610.

Then:
- Backfill R0640–R0646 metrics into the registry notes (no new rows).
- Add row `T0015c`, which corrects T0015's "ring ≤0.7" to 1.78°. T0015's own row is not edited.

**2b Code**
- Set the parameter default in `payload_betaflight_comm.py:74` and `tejen_betaflight_communication.py:56`. Set the env default in `src/tejen_mission/launch/m2d_four_drone_sequential.launch.py:344` to `"10.0"` (log it in authority.md §6).
- Port `betaflight_communication.py` (l.40, 150–156) and `angle_betaflight_communication.py` (l.31, 174–180) to `RatePid`.
- Add a shared gate `rate_pid.integrate_active(armed, u, u_min)`. The clamp stays at 200 and `rate_i_min_u` at 0.09.
- Tests: `pytest src/simulation_communication/test/test_rate_pid.py` plus one new `integrate_active` test. Run `py_compile` on the legacy bridges. Skip the package's flake8 tests, which lint Tejen's code.
- `drive_m2_handover.py`:
  - set `os.environ['MDC_RUN_DIR']` before `launch_ours()`;
  - argv[3] becomes sim seconds of hover, counted from `/payload/motion_capture_state` stamps or read-only `/clock`. This replaces `spin(hover_s*4)`;
  - start a record-only `ExperimentRunner` at the success hold;
  - drop `M2_LIVE_KI`.
- Write the patch: `git diff -- src/simulation_communication src/tejen_mission/launch > results/backups/step2_iterm.patch`.
- Note in CURRENT_STATE that SIL (`tools/sil/plant.py`) has no PID and now differs from Gazebo. No SIL runs at this step.

**2c Re-flies (variable: sim rate_ki 0 → 10)**

| Run | Config / command | Baseline | Falsifier | Wall |
|---|---|---|---|---|
| R0647 | `ocp_hover_ground_creep_defaults` (floor creep) | R0587 (0.6001, tilt 0.04) | z off 0.600 by >1.5 cm over the last 15 s, tilt >2°, lift >3 s later, any fault | 4 min |
| R0648 | `detach_hover_n4_3915` | R0592/93 | peak >4.5°, settled >1°, abort | 5 min |
| R0649 | `attach_circle_n3_ocp` | R0594, R0589 | no weld, peak >12°, abort; record pre-weld circle tilt | 5 min |
| R0650 | `partner_attached_orbit` (M1; also H1) | R0646 | fault, missing stage, reweld peak >10.4°. H1 is supported if ≤5° | 11–13 min |
| T0016 | `drive_m2_handover.py results/m2/T0016 false 20` | T0015/T0015c | join <4/4, stall, climb >45 s, any fault | ~21 min |

**Stop rule:** if R0647 fails, fly nothing else.
1. Re-fly R0647 with `SIM_RATE_KI=0` to confirm the I-term is the cause.
2. If it is, fly one arm with `rate_i_min_u` 0.15 through a test-copy arg. Carriers on the ring ramp to u 0.136–0.140 on the floor, so the 0.09 gate integrates during that ramp.
3. If that also fails, park: set the default back to 0 and ask Wesley in the report.

If T0016 fails, park it with the log. The T0015 path (`M2_RATE_KI=0 M2_LIVE_KI=10`) stays as the driver line.

**R0650 read-back (0 runs):** compute the ring tilt implied by the drone references. In R0646 the carriers sat within 1 cm of their references while the tilt held at 7–10°, so the tilt is commanded (H4).

**Exit:** 5 rows plus a milestone `/report`; then step 3.

### Step 3: G5 benches (build order: weld bench → M1-lite → M2 bench)

**Options**
- Full M1 (11–13 min) and full M2 (~21 min) as the only beds: too slow for 5–10 runs per hypothesis. Kept for claims only.
- The old near worlds (`three_attach_*_near.sdf`): out. They never welded (R0529–R0539) and use the old 0.05 m pivot.
- **Chosen: cut the partner mission from the attached start.** It runs R0646's exact code path, with only his mission removed.

**Weld bench, `configs/experiments/m1_rejoin_hover.yaml`**
- A copy of `partner_attached_orbit.yaml` with `load_traj: hover`, and without LAUNCH or `launch_expect_topics`.
- Events: ARM 3 → TAKEOFF 5 → WAIT_LIFT 0.5 → DETACH 3 at lift+5 → WAIT_PARTNER_RELEASE → HANDOFF → WAIT_REWELD → LAND at +12 s. Set `stop_after_landed_s: 2`.
- LAND is at +12 s because R0646 took 9.4 s to settle; LAND at +6 s would censor the settle metric.
- The approach starts from the step-out point, not his ATTACH_READY pose. Record the weld relative speed (R0646: 0.017 m/s).
- Wall ~4 min.
- Validation run (baseline R0650): the bench is out if the reweld peak is <5° while R0650 is ~10°.

**M1-lite, `m1_rejoin_orbit.yaml`:** the same with `load_traj: orbit`, `traj_speed 0.125` and DETACH at lift+10. Use it only if the weld bench fails validation. Wall ~4.5 min.

**M2 bench**
- Time box: 2 h of build plus 2 validation runs, then park. G2 claims are always full runs.
- `tools/sim_test/make_m2_bench_world.py` imports `m2c_ground_spawn` read-only and writes test copies to `simulation_assets/tejen/bench_m2/` (kept out of G6).
- World contents:
  - the 0.86 kg dynamic-ring patch (`run_m2d_sequential_attachment.sh` l.507–520);
  - the hold pose from T0015: plates 3/0/6/9, ring yaw 20°, bodies at z 0.602–0.605. Place each rod tip on its plate and assert the gap is <5 mm. Do not copy ring z 0.10, which is join drift;
  - hangers copied from `m2a_x3_support.sdf`;
  - absolute include URIs.
- `controller_quad_load/launch/m2_bench_io_launch.py`:
  - a copy of his launch l.280–345, with `rate_ki: 10.0`;
  - ros_gz bridges for `/bench/hanger_i/detach`;
  - declares `num_drones`, `attach` and `rviz`;
  - then `colcon build --packages-select controller_quad_load`.
- `configs/experiments/m2_bench.yaml`:
  - `dissipative_launch partner_m2:=true sim_interface:=false`, azimuths 90,0,180,270;
  - `cmd_ready_topic: /drone_{i}/ELRSCommand_diss`;
  - events: ARM → `/fleet/handover` → TAKEOFF → HANGER_RELEASE at +0.05 s → WAIT_LIFT → hover 10 s sim → LAND.
- Falsifier: drone tilt >25° or ring tilt >3° within 3 s of release. If it fires, the bench is used for lift, hover and LAND only.
- Wall ~4 min.

**Exit:** the weld bench is validated or M1-lite is chosen, and the M2 bench is validated or parked. [2026-09-27: M2 bench validated for lift/hover/LAND (R0707/R0708, falsifier not fired: drone tilt <=3.8, ring 0.0 within 3 s of release); it does NOT reproduce the M2 rock from rest] That is 2–4 Gazebo runs.

### Step 4: G3 attach transient (H4 first)

**Evidence (0 runs, offline with the real `blend_refs`/`balanced_tensions`)**
- The moment on the ring is t_new × (the distance of the newcomer's line from the incumbents' apex). At a=0 that is 0.25 N·m with a 1 N start tension and 0.025 N·m with 0.1 N.
- SIL R0546 (0 N start) plateaued at 1.7°. SIL R0578 (1 N start) plateaued at 7.4°.
- H3 is minor: R0594 welded at 0.127 m/s and still peaked at 10.4°.
- H5 is ruled out (the mux switched 12.5 s before the weld). H6 is cleared (the pivot is at the body centre). H2 affects ripple only.

**Options**
- **A (chosen first): the newcomer's start tension.** Add an opt-in param `attach_t_start_new`, default 1.0, in `dissipative_node._attach_ocp` (l.1021), plus a launch arg in `three_attach_launch.py` next to `attach_ff_ramp_s`. Cost: 3 lines and one 1-min SIL run.
- **B: swing, then load.** Add an opt-in `attach_blend_mode: swing_first` in `ReferenceBuilder._refs_at`, with a unit test that the moment is ≤0.03 N·m on a grid of a. Cost: ~40 lines. Use it only if A leaves a peak >2° or a slack rod.
- **Out:** rebalancing only the incumbents (residual 0.238 N·m remains) and a shorter blend (same peak moment).
- **After A/B:** universal joints (H2).

**Setup:** `python3 tools/prebuild_planner.py`, and check that the launch files are symlinked into `install/`.

| Run | Bench / variable | Baseline | Falsifier | Wall |
|---|---|---|---|---|
| SIL-1 | `configs/sil/attach_hover_ocp.yaml`, current tree | R0578 | <3° means the tree changed; re-read before going on | 1 min |
| SIL-2 | + `attach_t_start_new:=0.1` | SIL-1 | peak ≥6° → try B | 1 min |
| SIL-3 (if SIL-2 >2° or slack rod) | `swing_first` | SIL-1 | >4° → park H4 | 1 min + 1 h code |
| Gz-1 | weld bench (or M1-lite) + the best SIL arm | step-3 validation run | peak >7° or newcomer flip. Pass: ≤5°, dip ≤0.03 m, <2° within 3 s | 4 min |
| Gz-2 (if ripple >2° after 3 s) | universal joints, damping 0.002, test copies of the world and model | Gz-1 | ripple rms unchanged → park H2 | 4 min |

**Rules**
- Two runs without effect means park.
- Once a winner is found, the loop may shorten `attach_traj_hold_s` from 10 to 4–5 s in config copies (one run), provided the peak stays ≤5°.
- No critic is needed: this is reference shaping inside the existing OCP.

**Exit:** a winning opt-in arm, or parked with the log.

### Step 5: G1 claim with the G3 fix, then G1b

**Options**
- Judging by eye: out.
- Claiming on the bench: out, because the bench has no ball stages.
- **Chosen:** a `partner_attached_orbit` config copy with the winning arm, scored by `m1_stage_metrics`.

**G1 claim**
- Card: `docs/experiments/<date>_m1_claim.md`.
- 2 headless runs of 11–13 min each, baseline R0650.
- Falsifier: weld peak >5°, carrier dip >0.03 m, >3 s to get under 2°, detach peak >8°, orbit mean >3°, ball off the footprint at LAND, any abort, or TAKEOFF→LANDED >140 s.
- A result just outside a bar with a clear cause is a note, not a failure.
- Triage, one run per fix:
  - lift fails: rerun with `SIM_RATE_KI=0`;
  - pickup loops: `pickup_attach_clearance` 0.09;
  - transit stall: freshness 0.6 s;
  - no weld: log it and do not loosen the 0.05 m/s gate;
  - weld tilt: back to G3.
- If G3 parked without a fix, fly the claim anyway and report the rejoin miss.
- After 2/2: reviewer, then notify Wesley for a GUI run.

**Speed (after the claim, one exploratory run each, config copies)**
- DETACH at LIFTED+6.
- `pickup_lift_speed` 0.12→0.25. It is hard-coded, so it becomes a launch arg first.
- `stop_after_landed_s` 2.

**G1b**
- Chosen first: mass only. Test copies `simulation_assets/tejen/payload_model_m100.sdf` (0.1 kg, I = 2.56e-4) and a world copy, plus `partner_attached_orbit_m100.yaml`. Cost: 1 run, then a second run for the claim. Baseline: the G1 claim runs.
- Only if drone 3 sags >0.08 m or a stage fails: his UKF feedback, opt-in, with a `## Critic` in the card.
- Only if the 5 s return is missed: a `z_ki` config copy.
- **Exit:** 2/2 with the reviewer done, or parked with the log.

### Step 6: G2 M2 claim

**Options**
- Count T0015: out (no card, 1.0 s staleness gate, ~6 s sim hover).
- The bench: out for the claim, because join 4/4 is part of it.
- **Chosen:** T0016 counts as claim run 1 only if the M2-path md5 fingerprint is unchanged. The fingerprint covers `controller_mpc.py`, `planner_node.py`, `planner_solver.py`, `dissipative_node.py`, `elrs_mux.py`, `handover_policy.py`, the 4 bridges, `dissipative_launch.py`, the runner and the driver. Steps 4 and 5 will probably change it, so budget 2 fresh runs.

**Runs**
- Card: `docs/experiments/<date>_m2_handover.md`.
- Command: `tools/clean_slate.sh; python3 tools/sim_test/drive_m2_handover.py results/<date>/T00NN_m2_claim false 20`, ~21 min each. Scored by `summarise_m2`.
- Falsifier: any envelope fault, join <4/4, ring z outside 0.55–0.65 (from lift + 5 s), ring tilt >5° before touchdown (was 3°, Wesley 2026-09-28), drone tilt >25°, or no clean LAND. A lift overshoot is reported, not judged.

**Exit:** reviewer on a table with T0015c as the exploratory column, then notify Wesley for the GUI run (`… true 20`, ~25 min).

### Step 7: G4 review and G5 speed, alongside the other steps

**G4 candidates**
- **#1 `kt_hat` stuck at 83.10 in M2.** Read it for free in T0016. If it is still stuck, add a temporary debug print of the gate terms on the M2 bench, then delete it. Changing the steady band counts as a gate change: opt-in plus Wesley.
- **#2 staleness gate on the wall clock.** Opt-in `ref_age_clock:=sim` in `controller_mpc.py`. Needs a critic card, 1 SIL run and 1 bench run. The default is unchanged.
- **Parked:** #3 planner tick cost (CPU starvation), #4 LAND lag, #5 reference node 0.
- **No change:** #6 mux/policy (clean 4/4 in T0015 and W-21:36).
- **Tested under G3:** #7 cable-FF gate.

**G5 speed**
- The benches.
- Split the 58 s exit (stop grace vs plots) using log stamps.
- Profile the 94 s start-up.
- Opt-in rate cuts for his non-control nodes only, each checked by one full join and logged in authority.md §6.
- Never throttle `/clock`. Never pause the world mid-flight (the "LAUNCH after LIFTED" idea is dropped).

**G4 finding (2026-09-27, R0653): height integral idle in M1.** `_zbias_gated` freezes whenever `traj_t > 0`
(frozen through a circle by design, card 2026-09-24_planner_offset); M1 starts the orbit right after the lift, so
z_ki never runs and the ring flies at 0.66 (target 0.60) through the whole M1. Candidate (gate change -> critic
card first): integrate during a constant-speed level orbit (no vertical trajectory component, same taut/speed
gates), or seed the orbit with a hover-integration window after the lift (DETACH already waits lift+10 s).
Matters for G1b (height back within 0.03 after the drop).
TRIED 2026-09-27 (card 2026-09-27_zki_orbit, critic applied): R0676 fixed the height but stepped the detach (11.4) and
weld (5.3) and neared the bound -> falsified, code deleted. Parked; ideas in the card.


**G4 finding (2026-09-27, reviewer on the combined M1 claim): carrier tracker kicks.** A carrier's MPC occasionally
steps thrust (0.167 -> 0.232) and pitch rate (-> -2.3 rad/s) within one 20 ms step with smooth references and no solve
failure (R0685 x2, R0686 x3, R0691, R0696 at 47.5 s, R0697 at weld+5.1 s): 1.6-4.6 deg ring excursions. Cause open:
candidates a solver warm-start jump/QP active-set switch, a stale pose/rate sample under CPU load, the kt_trim or
cable-FF update. Next: dump the tracker's solver status/iterations and inputs around one kick (log only).
Update: solver status 0 throughout (R0697 drone 0); the kick starts as a one-sample jump of the MEASURED body rate
(wy -0.17 -> -2.28 rad/s in 14 ms) during a sustained -0.33 rate command, and the thrust/yaw kick follows. Not the rate
I-term: one-step |d omega| > 1 rad/s also occurs in the ki 0 runs (R0640 x2, R0642 x3) as often as with ki 5.
Next: log the raw pose stamps / the rate source around a kick (pose glitch vs real). Parked (within every bar).
Tried 2026-09-27: the sim mocap emulator differenced poses over the node clock at callback time (bursty under load);
opt-in MOCAP_STAMP_DT=1 differences over the message stamps: kicks 5/8 (R0700/R0701) vs 2/10/13/26 without (R0696-R0699)
-> suggestive, not conclusive. Then the unthrottled-clock M1 (planning Q5) gave 183 kicks and a speed fault (R0702, clock
stays 500 Hz), and with the stamp fix still 178 (R0703) -- BUT the bridged PoseArray has no stamp (R0536), so that fix
never engaged: the test was void (row R0703c). Both the mocap emulator and the sim Betaflight bridge difference poses
without real sample times (the bridge uses a 0.25 s average period), so bunched/dropped samples under load make rate
spikes. Proposed fix (Wesley): rate loop on the X3 IMU gyro (bridge the IMU), like a real Betaflight.

**Tejen's review of the R0687 logs (2026-09-27) -> loop items:**
1. Post-pickup climb: our partner_mission.launch.py override (pickup_lift_height 1.40, minimum_search_z_m 1.8, set
   because his C++ transit needs z >= 1.8, R0639). Ask Tejen what the 1.8 m floor protects; lower both if possible
   (~10 s per M1).
2. Handoff blip (OURS, confirmed R0687): ApproachProfile always starts with 'climb' to 0.25 m above the tip-on-target
   height; at ATTACH_READY drone 3 is already 0.69 above the ring, so the reference jumps +7.5 cm (vz +0.22) before the
   descent. Fix: start in 'descend' from the measured state when handed over within the clearance (opt-in, then default).
   DONE (opt-in attach_approach_direct): R0698 reference rise +0.021 (was +0.09-0.11), no upward speed, weld 1.9 s after
   the handoff (was 2.8-3.0), every combined bar passes. Default: Wesley's word (with the other opt-ins).
3. His DESCEND_TO_ATTACHMENT vz overshoot + sharp upward correction: measure across the existing M1 runs (his logs)
   before any tuning; candidates his vertical model vs our linearised plant, or the bridge I-term.
4. Collision safety in the partner segment: DONE as a metric (metrics.py partner_clearance: partner drone to each
   carrier body >= 0.30 m and to each carrier rod >= 0.15 m, per segment, in every M1 metrics). Our step-out
   0.51-0.53 m, our rejoin >= 0.48 (rod) / 0.52 (body); HIS mission segment comes closest: 0.29-0.35 m to a carrier
   body (R0687 0.294 < 0.30) -> for Tejen (his collision avoidance / keep-out around the carriers).
5. Gates (M2 settle/handoff/freshness): his d9c6941 (IRL gate profiles) is NOT in our fork (fork base a7524bae).
   Merge a7524bae..his final commit in one go after his state-machine renumbering (authority.md), then his tests + M2 + M1.

### Questions for Wesley (planning pass)

1. **Will you run the step-1 commit before the loop reaches step 2?** Recommended: yes. If you don't, the loop carries on and `step2_iterm.patch` lets you split the commit later.
2. **Rate-loop scope.** Should the I-term default also go into Tejen's forked `m2d_four_drone_sequential.launch.py` env? And if the floor start fails, may the loop raise `rate_i_min_u` 0.09 → 0.15, at most once, through a test-copy arg first? Recommended: yes to both. Without the first, M2 stays P-only by default.
3. **The partner at-rest weld gate of 0.05 m/s** (`three_attach_launch.py:689`, the R0646 change) went in without your word. Keep it in the G1 claim baseline? Recommended: yes.
4. **Promoting the G3 fix.** May the loop fly the winning opt-in arm (e.g. `attach_t_start_new` 0.1) in the G1 claim config copy, and make it the `dissipative_node` default after the claim and the reviewer? Recommended: yes to both.
5. **M1 throttles `/clock` to 500 Hz** (`clock_hz: 500`), against the rule that /clock is never throttled in his world. His transit logs 9–17 tube violations per run. May the loop fly one M1 config copy with `clock_hz` 0 and adopt it if it is cleaner? Recommended: yes.
6. **Staleness gate on the sim clock (G4 #2).** May the tracker measure reference age on the ROS clock under `use_sim_time`, as an opt-in first with a critic before it becomes the sim default? The rig is unchanged. Recommended: yes. It would replace the 3.0 s sim workaround.

### Answers to the planning-pass questions (Wesley 2026-09-26, before the loop)

- Q2 I-term in Tejen's launch env: **yes**. Raise `rate_i_min_u` 0.09 -> 0.15 once if the floor start winds up: **yes**.
- Q3 keep the 0.05 m/s partner at-rest weld gate in the G1 baseline: **yes**.
- Q4 fly the winning G3 arm in the G1 claim and promote it to the default after claim + reviewer: **yes**.
- Q5 one M1 copy with `/clock` unthrottled, adopt if cleaner: **yes**.
- Q6 staleness gate on the sim clock, opt-in + critic first: **yes**.
- General: "The loop is free to do whatever it can to help achieve the goal as best as possible."
- **Commits (changes G6 and the no-commit rule for this loop):** Wesley asked the loop to commit.
  Done 2026-09-26: branch `integration` created from `week1-ready-for-testing` (35c2363, unchanged, now
  clean); commit d8aea49 holds all the uncommitted work (.gitignore, tests.txt, src/, simulation_assets/;
  his logs ignored). The loop keeps committing its own work on `integration` at milestones (source and
  worlds only; authored by Wesley's git identity, no trailers). Never push,
  never touch week1. Gitignored tools/docs/configs are backed up in `results/backups/pre_loop_2026-09-26.tgz`.

## How the loop works (Wesley is AFK: it never stops to ask)

- **Before the loop:** one planning pass produces the concrete plan for every goal and asks Wesley all its
  questions at once ("Questions before the loop" below). Only then does the loop start.
- **During the loop:** an item that needs Wesley's word is never guessed and never blocks. The loop writes
  the question under "Questions raised during the loop" with its evidence and recommendation, does the work
  only in a reversible form (a test copy, an opt-in launch arg / env flag, a separate config; defaults,
  world SDFs, params.py and gates stay untouched), and moves on to the next item.
- **Notify** (push) only at milestones: a claim passes, a goal is done, a GUI run is worth watching, or
  every remaining item is parked on a question.
- **Survive usage limits / restarts:** every iteration is idempotent and takes its state from files, not
  from the conversation: this file (status, questions, parked), results/registry.csv, the run directories.
  A run in flight is recognised from its directory and processes (never from a notification: background
  tasks are not restored after a restart or limit). Start of each iteration: `tools/clean_slate.sh` unless a
  run of this loop is still alive, then continue with the next item. Live-run check: headless Gazebo shows as process
  name `ruby` (not `gz`): `ps -eo comm | grep -xq ruby`, plus the newest run dir without metrics.json.

## Method: step back before digging (no rabbit holes)

- Start of every goal and every new sub-problem: write a short **options note** (in the experiment card or
  here): at least three approaches, what each costs (runs, code, risk), what result would rule it in or out,
  and why the chosen one is the cheapest discriminating step. Prefer the smallest bench that can show it.
- **Time box:** two runs without the expected effect -> stop, re-read the evidence, rewrite the options
  note, and either switch approach or **park** the item (with its log and next idea under "Parked") and
  move to the next goal. At most ~6 runs on one hypothesis before parking it.
- One variable per experiment; a claim needs its repeats (working rules); delete code that did not beat the
  baseline the same session.
- Keep G1/G2 primary: a side investigation that does not serve them within ~2 hours gets parked.

## Answers before the loop (Wesley 2026-09-26)

1. Object mass for G1b: **~0.1 kg** (a steel ball of that mass is ~0.029 m across; keep the sim geometry,
   set the mass, note the geometry mismatch).
2. Thresholds: **use the ones above, but they are approximate**: "as good as possible, within reason".
   A run just outside a threshold with a clear reason is a note, not a failure to chase for hours.
3. Tejen: nothing new; the forked code is the baseline.
4. Priorities: **stability during the join/rejoin first**; **speed matters everywhere else** (demo time,
   sim wall time, test turnaround), because faster runs mean faster development.

## Questions raised during the loop

1. Adopt the rod model with universal joints at both ends (x3_drone3_magnet_rod_partner_attached_uu.sdf) as the
   partner drone's model? It is a sim workaround for a gz weld artefact (spin copy) and removes the rod's twist
   freedom. Recommendation: yes for sim (M1 claim R0685/R0686); check the real rod's freedom separately.
2. Flip the four opt-in rejoin flags to defaults (attach_t_start_new 0.1, attach_blend_balanced true,
   weld_velocity_clock sim, attach_datum_shift true)? Recommendation: yes (bench + M1 claim), each is reversible.
3. G1b (0.1 kg object): enable his thrust_ratio UKF (an estimator in his flight loop) or schedule his kT on
   /tejen/object_attached (82.7*m_d/(m_d+m_obj) while the object is attached, 82.7 after the drop; small change to
   his code)? Critic on the UKF card (2026-09-27_g1b_mass.md): the UKF recovers too slowly after the drop (~20-30 s)
   for the rejoin. Recommendation: the scheduled kT (deterministic, exact at attach and drop).

(none yet; the I-term default was decided by Wesley 2026-09-26, decisions.md l.41, and his launch env on the planning-pass answers)

4. Sim plant: drive the sim Betaflight rate loop from the X3 IMU gyro (bridged) instead of differencing unstamped poses?
   Likely removes the carrier kicks (they scale with clock rate/load, R0702). Changes every sim baseline; opt-in first.
   Recommendation: yes, opt-in + bench + one M1/M2 each, then decide.
   DONE as opt-in (1093b89): R0705 no clear effect vs a typical pose run (kicks 7 vs 7, ring p99 2.04 vs 2.16).
   Wesley: keep the opt-in (realism, harmless when off) or delete (working-rules default)?
   ANSWERED (Wesley 2026-09-27): keep, off by default (rate_source pose stays the default).
5. M2 on the gyro (2026-09-27 late): make rate_source imu the sim default on his bridges (M2 path), and judge the M2
   claim on the sustained rock instead of any sample over 3 deg? T0028/T0029 fail the bar only on short transients
   (3.44 lift-off, 3.33 LAND onset; >2 deg for 0.3-0.5 s, >3 deg ~0.1-0.2 s; T0018 had 2.38 at LAND onset on the pose
   path). Recommendation: gyro default for M2 (matches the real Betaflight); the bar is his call (a 0.5 s mean would
   pass both, ~2.5-2.6). If the bar stays, next: the payload mocap path under load (pose rows repeat 0.13-0.37 s).
   ANSWERED (Wesley 2026-09-28): gyro default on his bridges AND ours (sim-only); M2 ring-tilt bar 5 deg per sample. Re-fly baselines once, M2 claim 2/2 on fresh runs.
6. Abort behind the mux (G7 P7, card 2026-09-28_mux_abort_latch.md, critic NOT READY): (a) may OUR fleet abort ground
   Tejen's drones while they still fly under his MPC (M2 before the hand-over, M1 during his mission)? Today it cannot reach
   them. (b) Should an ARM failure of our stack raise /fleet/abort at all (today it does, main.py:246; in M2 we ARM while
   his fleet holds the ring airborne)? Recommendation: (a) yes for an operator ESTOP and a flying-drone fault, (b) no:
   ARM-failed should refuse TAKEOFF (it already does) without the fleet-wide abort. Not needed before Wed (no mux in R0-R4).
   ANSWERED (Wesley 2026-09-28): (a) yes, ESTOP + flight faults ground every drone incl. his; (b) no, ARM-failed refuses TAKEOFF without /fleet/abort.
7. Rig-mode defaults (G7 P8 review): in real:=true on three_attach_launch / dissipative_launch, default to the creep floor
   start (start_taut false, handover_elev_deg 45, handover_settle_s 2.0, creep_vel 0.2) and reconfig_mode ocp, as a
   real-only substitution that keeps any typed value? Today real mode keeps the sim air-start defaults and only warns;
   the Wed sheet types everything explicitly, so nothing breaks before R8. Recommendation: yes.
   ANSWERED (Wesley 2026-09-28): yes, rig-mode defaults to the creep floor start and reconfig_mode ocp (typed values honoured).
8. Rig magnet path (G7 P10, built + reviewed GO, UNCOMMITTED in the working tree pending this answer): in real:=true
   every drone's magnet has ONE path, the radio String latch /drone_<d>/magnet (defined from boot); carriers ON, the
   newcomer's boot state `attach_magnet_initial:=auto` (ON when partner_attached, else OFF; was ''), '' refused in real
   mode, `weld_radius:=0` = dry approach (manager drives no latch); detach_magnet true (a detach releases the magnet).
   Sim graphs identical (148 cases). These are rig-mode launch defaults -> your word. Also for Tejen (M2b-M2d): with our
   graph owning the radios his magnet commands no longer reach the magnets (ON from boot through his join); if his
   elrs_interface_irl owns the radios (real_io:=false) our graph reaches no magnet. Recommendation: commit; settle the
   M2 magnet ownership with Tejen before M2b.
   ANSWERED (Wesley 2026-09-28): commit it; M2 magnet ownership to agree with Tejen before M2b.
9. Floor-start M1 (G7 P15, the sim twin of rig R11): the M1 world already starts on the floor; the air start is
   start_taut true. With the creep (start_taut false) the dissipative node leaves drone 3 out of the creep (the creep
   controller is sized n=3 and _do_attach refuses before the planner phase) and then folds it in at the hand-over with
   a weld point from its body on the floor (~0.4 m below the ring) instead of plate 3 (the plate override is gated on
   'not takeoff_seen', dissipative_node.py:1030). Fix options (attach geometry -> your word): (a) minimal: key the
   plate override on the first attach of a start_attached drone; drone 3 then joins at the hand-over with the right
   geometry but is swung up by the OCP instead of creeping; (b) rig-faithful: resize to n=4 before TAKEOFF in the creep
   phase too and size the creep controller n=4 so drone 3 creeps as the 4th carrier. Recommendation: (b), it is what
   R11 flies. CONFIRMED by R0732: drone 3 folds in at z 0.20, hangs ~0.35 m below the carriers, ring tilt 8-13 deg,
   detach 9.68 / rejoin 5.23 (bars 8 / 5).
   ANSWERED (Wesley 2026-09-28): (b) drone 3 creeps as the 4th carrier.
10. Carrier kicks (parked G4, all runs within bars): the data now favour glitches in the body rate fed to the tracker's x0
   by the mocap emulator (M-kicks2; 78 vs 1.45 kicks per 1000 ticks for glitch-only vs stall-only), the same arrival-time
   differencing the rig mocap publisher uses (P9). Pursue now (one or two clock-0 runs with a gyro-fed x0 rate as a sim
   oracle, card + critic first), or leave parked until after the rig week? Recommendation: leave parked; decide P9 from
   the R0b bag on Wednesday, which is the rig-relevant half.

   ANSWERED (Wesley 2026-09-28): pursue now with the sim oracle test (card + critic first; 1-2 clock-0 runs approved).
   ANSWERED again (Wesley 2026-09-29, after T0035 failed the M2 tilt bar on a lift kick): fix from the R0b bag (P9 decides
   the rig rate estimate; then the same fix in sim); meanwhile the M2 claim reads "met when no kick lands in the lift".
11. Pre-TAKEOFF tracker disarm (card docs/experiments/2026-09-29_pre_takeoff_disarm_gate.md, critic NOT READY -> v2):
   today a tracker that disarms between ARM and TAKEOFF (pose watchdog under load, R0749c; on the rig a mocap dropout
   > 0.25 s) is ignored and TAKEOFF goes ahead: the other drones lift the ring one-sided, and in three_attach drone 3
   takes TAKEOFF straight from /fleet/command. Proposed: treat it like a failed ARM (Q6b): service-disarm our fleet, no
   /fleet/abort, refuse TAKEOFF. Cost: the trackers shut down, so a floor mocap dropout means a relaunch instead of a
   re-ARM. Build it? (a) yes as v2; (b) yes, but keep the trackers alive (refuse TAKEOFF only; drone 3 in three_attach
   would still need its own gate); (c) not now.
   ANSWERED (Wesley 2026-09-29): (a) ground our fleet (service disarm, no /fleet/abort, refuse TAKEOFF).

## Parked
- **Carrier kicks, 2026-09-28 update (M-kicks):** kicks line up with x0 body-rate glitches (62-70 % vs 0.1-2.3 % of ticks;
  ~half inconsistent with the pose attitude path; both ~8x with the unthrottled clock). Same arrival-time differencing as the
  rig mocap publisher (P9). Next when unblocked: card + critic for an opt-in >= 20 ms differencing window in the sim emulator
  (one clock_hz 0 run vs R0702/R0703), then the same design for the rig publisher (P9). x0 velocity now logged.

- **M1 rejoin yank (2026-09-27, R0667-R0675).** Every bench with the full fix set is clean (hover R0665/R0666,
  orbit R0672, orbit + pickup-joint model R0673, orbit welded after 85 s R0675: 1.6-1.9 deg, dip <= 0.033, never
  above 2 deg). Full M1 with the same fixes (partner_attached_orbit_g3_ds) yanks the ring AT the weld: R0668 23.5,
  R0669 32.3, R0670 36.4, R0674 22.8 (dip 0.15-0.30). Evidence: no weld snap (tip fixed at (-0.007,0.282,0.049) in
  the ring frame across the weld, R0674 stream); tip spin 1.2 rad/s (negligible); all carriers' xy refs step
  (-0.05,-0.02) m together in 0.18 s after the resize (planner-side) while the bench refs do not; same resize log,
  datum shift, ref ages, no slow ticks; ball not attached; his magnet manager did not attach; no disarm. Only the
  partner mission differs. Next ideas (each one run): (1) opt-in debug dump of the resize inputs (x_init incl. drone 3
  velocity, t_before, first solution) in M1 vs bench; (2) M1 with his mission stopped right after the handoff (his MPC
  and supervisor still publish: /join_planner/*, his pendulum/transfer nodes); (3) check what else subscribes/publishes
  /magnet/object_attached, /partner/release, /payload/trajectory_state in M1 at the weld.
  G1 still has the full chain with ball in net and ~105-115 s TAKEOFF->landed; only the rejoin bar fails.
  Update (R0677-R0679): resize-input logs show the M1 ring already moving at the resize (angular 0.2 rad/s; the
  resize lags the physical weld), OCP inputs otherwise alike; the yank persists with his whole launch stopped at the
  handoff (R0679, 15.4): it is the STATE his mission leaves drone 3/the world in (candidates: drone 3's bridge
  rate-integrator after carrying the ball, the pickup DetachableJoint's attach/detach history on the tip link,
  drone 3's approach from ATTACH_READY). Side finding: his object DETACH via one-shot gz CLI drops (ball stayed
  welded after DROP, R0678) -> backend both + ros_gz bridges (partner_mission.launch.py).

- **G1b (0.1 kg object) stalls in his LIFT_OBJECT (R0681).** Drone 3 carries the ball 0.5 m below his lift
  height (his MPC: no z integral, no object mass; thrust_ratio UKF off and gated at throttle >= 0.15 while the loaded
  hover is 0.137). Options: (1) his `enable_thrust_ratio_ukf` with `thrust_ratio_ukf_min_throttle` 0.10 (an estimator
  in his flight loop: card + critic first); (2) a payload-mass feedforward param in his MPC (his code change);
  (3) a relative lift threshold in his planner. Also: the shipped ball inertia 0.002 at 0.01 kg is 78x a solid
  sphere (test copy uses 2.56e-4).

- **M2 ring rock (2026-09-27, T0017/T0020).** Intermittent 1.27 Hz rock of the ring/two opposite drones during our
  lift/hover (T0020: 7.07 deg lift, 4.0 hover; 2/3 pass at ki 5). Not a code change since the claim. Suspect his
  ball-jointed tethers (undamped in DART) but universal tethers break his flight (T0021: drone 0 at 90 deg after his
  join; pivot 0.05 m below the body + blocked twist). Ideas: universal at the RING end only (keeps drone-end twist and
  swing free); a damped yaw-free chain (extra link, changes his pose-array indices); or accept + report the rate.
  Ring-end universal tried: T0022 pass, T0023 join stall, T0024 rock 16 deg -> refuted (removed). Record so far at
  ki 5 on his shipped model: good T0018, T0019; bad T0020 (and T0017 at ki 10). Next (step back): an M2 hand-over bench
  (G5, skips his 15 min join) to get enough repeats to find what differs between rocking and calm runs (hand-over state:
  rod elevations, drone yaw, ring yaw, creep latch; the lift profile).
  Diagnosis workflow (M-rock): hand-over state identical in good/bad; the rod gate DAMPS (not the cause); rim point
  3.7 cm below the real magnet (attach_z) -> corrected in T0026/T0027: still rocks (12.96, 21.42). No M2-path code
  change since the claim. Tally since the claim: rock 4 (T0020, T0024, T0026, T0027), calm 1 (T0022), his join
  stalls 2 (T0023, T0025). Each M2 run costs ~25 min (his join): building the M2 hand-over bench (G5) next.

## Needs Wesley (never guessed: asked above, worked around reversibly meanwhile)

- Adopting damped universal joints in the real models (after the weld bench shows it helps).
- Any launch default, world SDF, params.py, gate threshold, attach geometry (working rules).
- Commit: the loop commits on `integration` (Wesley 2026-09-26); never push, never week1.
- Thresholds in G1/G2 if these are wrong.

## Budget and stop rules

- Headless Gazebo runs per loop session: **no cap** (Wesley 2026-09-26, waives the working-rules 6/session
  for this loop); report at milestones (a claim passes, a decision is needed, something to watch).
- A fix that has not shown its effect in two runs -> step back per "Method" (switch approach or park it,
  log in the registry and under Parked); the loop itself keeps going on the next item.
- Nothing else heavy runs during a headless run (wall-clock gates).
- Notify Wesley: a decision is needed, a claim passes, or something is worth watching in the GUI.
