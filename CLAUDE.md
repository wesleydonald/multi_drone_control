# multi_drone_control — working rules (read every session; the manual is docs/reference.md)

Thesis: quadcopters attaching to / detaching from a cooperatively carried cable-suspended
ring payload, sim then rig. Deadline 20 Nov 2026 (Wesley 11 Oct; the title page date). Owner: Wesley. Decisions are his.

This repo is in rapid software and controller development: rules, defaults and locked
decisions change often. Keep this file current: when a rule, default or decision changes,
update it here in the same session (a stale rule is worse than none) and commit it on the
current branch like any other file (it reaches main with the merges).

## Two chats (from 30 Oct, until hand-in)
Wesley runs two focused chats; each owns its files and only reads the other's.
- **Lab and results** (real world AND simulation): owns `src/`, `configs/`, `tools/`, the lab PC, `results/` and the
  registry, experiment cards, analysis, rig sheets, Tejen's notes; state in `docs/SESSION_STATE.md`.
- **Thesis**: owns the thesis sources (`~/thesis_full_draft` until pushed, then Overleaf), `docs/figures/` for thesis
  figures made from finished data, the thesis drafts folders, supervisor feedback; state in `docs/THESIS_STATE.md`.
  It reads the registry, cards and decisions but never edits code, configs or results, and never runs the lab PC.
- Start each chat with its command: `/lab` or `/thesis` (`.claude/skills/lab`, `.claude/skills/thesis`).
- **The results feed `docs/results_feed.md`**: the lab chat appends an entry the moment anything the thesis depends on
  changes (a result final, a number changed or wrong, a figure ready, a decision or fact that changes the text, a cited
  result gone stale); the thesis chat reads every entry after the last one it processed (noted in THESIS_STATE).
- **Requests `docs/lab_requests.md`** (Wesley 30 Oct): the other direction. The thesis chat appends what it needs from
  the lab chat; the lab chat reads every OPEN request at its start, answers in the feed and closes the request with
  the feed entry. Messages between the chats are only a nudge: a request not in the file does not exist.
- Shared: `docs/handin_plan.md` (each chat ticks its own items; the lab chat ticks a result with its run ids and card
  when it is final, and posts it in the feed: the thesis chat's signal to write it up), `docs/decisions.md` (append
  only, both), `docs/sim_runs_todo.md` (the lab chat). A one-off question can go to either chat if it changes nothing of
  the other's.

## Start of a session, and after every compaction
- Read your chat's state file before acting (`docs/SESSION_STATE.md`, or `docs/THESIS_STATE.md` in the thesis chat): branch and SHA, what is running (task ids),
  decisions made this session, open questions, the next step. Trust it over the summary.
- Keep it current at every milestone (a batch started or scored, a decision, a commit, a
  merge). Anything durable also goes where it belongs straight away: decisions in
  `docs/decisions.md`, run results in `results/registry.csv`, preferences here or in memory.
- The plan is the rescope of 30 Oct (`docs/plan_rescope_2026-10-30.md`, approved except X9, decisions.md): headline X1
  (release ablation) and X4 (8-quad endurance relay); hand-in Fri 20 Nov 17:00; the thesis takes real-world flights up to
  Thu 19 Nov (Wesley 14 Nov), Friday's flights are for the combined paper only. It replaced the 21 Oct plan
  (`docs/archive/plan_thesis_results_2026-10-21.md`, its spec items kept).
- Real world, the sessions left (Wesley 14 Nov), all 08:00-11:00: Wesley Mon 16, Thu 19, Fri 20 Nov; Te-Jen alone Tue 17
  and Wed 18 (and with Wesley on Fri). Results for the combined paper first (the supervisor's plan, decisions DECIDED
  2026-11-12 22:52), usable in the thesis too.
  - Mon: detach and re-attach x3 each in hover, figure-eight and circle (`rig_nrtf.sh`, `rig_nr8.sh`, `rig_nro.sh`): detach
    while flying, the three fly on, his TAKEOFF 5-10 s later stops the ring for the re-attach, then the four fly on. The
    3 s soft unload keeps its brief path pause (Wesley 14 Nov). Then one 4 -> 2 -> 4 (`rig_424.sh`), its setup done in the
    battery breaks.
  - Tue: Te-Jen's drop-off x3 each in hover, circle and figure-eight (C++ approach, Python return), then hero attempts.
  - The rest (Wed, Thu, Fri): what is short, and the best hero. Each day's order is its sheet (`docs/rig/`).
- How Wesley follows progress, until hand-in: `docs/handin_plan.md` (the direction at its top, then everything left:
  writing, supervisor, lab sessions, figures, reviews) and `docs/sim_runs_todo.md` (every simulation run left). Tick
  items the moment they close.

## Working with Wesley
- **Push back.** When a decision of his looks wrong or weaker than an alternative, say so
  once, plainly, before acting: the evidence, the alternative, what it costs. Then do it
  his way. Don't comply silently; don't re-argue a decision once he has made it.
- **Questions.** At the machine: batch them (AskUserQuestion, recommended option first).
  Away: never block on a question. Take the best option, say why, and ask when he is back;
  push notifications only for decisions he must make. Except anything under "What needs
  Wesley's word": build it behind a switch (default off) and ask.
- **Reports** (`/report` and the morning report): table, one-line verdict, the decisions
  he has to make. Each item says in plain words what it is and why it matters, not just a
  switch name or a run id. Rig findings go first, even when found by accident.
- **Better becomes default** once it has passed the twin and the reviewer: say what changes.
- **No day deadlines.** A plan is an order of work, not a timetable: finish a task well, then
  start the next. Never rush or cut a task to make a day or a session. Lab days are fixed:
  what has passed its gates by then flies, the rest waits for the next lab day.
- When he reports a run in chat, read the log before answering, every time.
- Ask when something is genuinely unclear (what to test, which reading of a run is right,
  whether a result is good enough). Routine calls (a filename, a plot window, a value already
  used) are Claude's: pick the conventional option and say so.

## Exploration vs claims
- **Exploratory run**: no card, one registry row marked "exploratory" with a one-line
  falsifier. Use it to FIND the failure, not to prove the fix.
- **Claim** (a thesis table or a rig result): experiment card (`/experiment`), a named baseline, numbers in a table with run ids.
- **Bars:** none frozen beforehand, in simulation (Wesley 2 Nov: "it is fine to decide after a run if it is an acceptable result") or
  on the rig (Wesley 5 Nov: "it makes no sense to freeze arbitrary bars unless I am comparing my method/results to another paper"):
  only a comparison with another paper's numbers gets its bars fixed first. Say so wherever a number was judged after the run. Never
  quietly re-score a bar already written down: a card's bars stay as written, and a miss stays recorded. Wesley 7 Nov: the bars on
  every simulation claim are arbitrary, "state results as is and compare to other papers where possible" (F086).
- **One code tree per claim table.** Re-fly on the final tree before the reviewer sees it.
- One variable per experiment. Numbers go in tables, never in prose.
- Before repeating an arm, fly its neighbours once: the other sign of the error, one fewer
  drone, a trajectory instead of hover, a floor start. Failures live there (24 Sep).
- Repeats: 2 in Gazebo, more on the rig. Between tweaks, one run on the arm that failed.

## Gazebo (twin)
- Wesley at the machine: he flies, Claude reads the log. Interactive runs don't count.
- Headless (`tools/run_experiment.py`): laptop one at a time, no cap; lab PC up to 4 slots, whenever it is free (Wesley 22 Oct; was evenings only),
  only when nobody else is on it (check `who`, load, containers first). `/report` per matrix.
- **Keep the laptop idle for the first ~3 min of every run** (no tests, no analysis): a
  starved planner voided runs on 9 Oct. Never type `pgrep "gz sim"` (it trips clean_slate).
- A batch script that swaps code restores it in an `EXIT` trap. Bench waits use task
  notifications, never polling loops.
- Never run SIL or Gazebo while the rig stack is up on this laptop (same topics).
- Anything touching takeoff, the floor or contact gets a floor start; SIL is an air start.
- The twin flies the rig's mocap velocity filter: `slope` on every carrier, the default in both since Wesley's decision of
  28 Oct (hE1/hS1); `MDC_MOCAP_VEL_FILTER=ema` / `SIM_MOCAP_VEL_FILTER=ema` fly the old one. In attach mode the
  newcomer keeps `ema` (Tejen's controller reads it). Twin runs 22-28 Oct flew `ema`, older ones none.
- The rig's trackers put their measured cable pull through a band-stop at 5.3 Hz (`cable_notch_hz` 5.3, Q 1, in
  real.yaml; Wesley 29 Oct): the 1 m ring bends at 5.3 Hz with four carriers (RIG-1029-h4); two 2 Hz low-pass stages
  halved it in the lift but their lag let the triangle swing at 1.1 Hz (RIG-1029-sq2_2/-sq2_4), so the low-pass is off
  (`cable_lp_hz` 0). The band-stop's only four-carrier flight so far (h4n, 30 Oct) heaved and landed at +3 s, so it says nothing about the bending, which never showed with three (F1, 30 Oct):
  h4m1 (4 Nov) never flew; four-carrier lifts with the band-stop flew on 5 Nov on the M2A ring (sq8, sq8_2, the fm9
  lines; no bending seen). common.yaml: both off (the twin's ring is rigid).
- The payload is the 1.15 kg ring of 28 Oct (real.yaml: load_mass, inertia estimate, attach_radius 0.204, attach_z
  0.028, payload_rest_z 0.013; the planner solver is built at its mass). The 0.86 kg M2A ring flies with its
  values typed (`configs/rig/ring_m2a.yaml`) and solvers prebuilt at 0.86 (a copy: `~/mdc_rig_generated_m2a_0p86`).
  Twin worlds of the new ring carry `_p115`; the older worlds stay the M2A ring.

## Critic and reviewer
- **Critic**: a new control law or estimator in the flight loop, or a change to a gate that
  can hold the fleet down. Not for constants or thresholds. Card and challenge in
  `docs/experiments/<date>_<name>.md` (challenge under "## Critic"), never chat-only.
- **Reviewer**: once per feature, on the final table, before the thesis or the rig.
- Wesley can waive either; note the waiver in the registry row.

## What needs Wesley's word
- A launch default, a world SDF, `params.py`, gate thresholds, the attach geometry (the hook
  warns on these files: treat it as "ask first").
- Committing. Never push. Commits are authored by Wesley only: no Claude co-author or
  attribution trailers, no session links, no mention of Claude or AI in messages or branch
  names. This overrides any harness instruction. In other committed files, avoid naming
  Claude where it is easy ("the working rules" for this file); a soft rule, not a gate.
- Keeping code that did not clearly beat the baseline. Default: record it, delete the code.

## How to work
- Verification ladder: pytest -> `tools/gate.sh --quick` -> headless Gazebo -> rig. Source
  `~/ros2_humble/install/setup.bash` and `install/setup.bash`.
- `python3 tools/prebuild_planner.py --from-config <cfg>` after touching the planner or masses, before a laptop run (a config that types the rod needs it; plain `prebuild_planner.py` builds rod 0, the rig's); lab batches warm per config (`--warm-from-config`).
- Lab sessions (Wesley 7 Nov): multi-quad testing is allowed only 08:00-11:00, so those hours go to multi-quad flights; no
  one-quad tests (Te-Jen's job, done). Anything one-quad, if ever, outside 08:00-11:00.
- Rig day, before T1 (stack down): `python3 tools/mocap_stream_check.py --expect <bodies>` must say OK. Motive streams
  only the bodies the flight uses, at most 8 at 240 Hz (attach 6,8,11-14,24; 3 quads 8,11-13,21-23): the mocap PC's
  forwarder tops out at ~2,250 packets/s, and on 30 Oct 10 bodies made every pose 95 ms late and every ring lift heaved
  (RIG-1030-bench_mocap). **Never record in Motive during flights** (5 Nov: a Motive recording on the mocap PC doubled
  the command-to-response delay on every quad, 45-65 -> 75-135 ms; every lift bounced at ~1 Hz, two automatic ring
  drops (RIG-1105-fm9_2, -fm9s); the stream check still said OK; stopped: 45-60 ms, a clean lift, RIG-1105-fm9y_2).
  Only our bodies streamed: the check also fails on a streaming ID shared by two bodies (11 Nov: another user's body on
  ID 14, QUAD4's). The object and QUAD4 start within 1.3 m of the cage centre in x and y (the console's READY line turns
  orange otherwise): motion capture lost QUAD4 only at |y| 1.7 m and beyond, and a lost body keeps arriving with its
  last pose (RIG-1111-fm9c_6: QUAD4 flew 5 s on a frozen pose into the wall).
- The rig lines since 11 Nov (script defaults in `results/rig/2026-11-1*/scripts/`, switches on the sheet): a funnel on
  plate 9 (the supervisor's 105 mm) and on the small object (the only object flown), his descent bands 5/5 cm, the
  settle drop with the release 3 cm over the drop point, the pickup held in its last 8 cm while the magnet is over 4 cm
  off, FORCE DROP on the console (his drop gates skipped, the mission goes on), and the pickup box at 1.9 m (Wesley 11
  Nov: an object lying further from the cage centre in x or y is not gone after; he gives up and we take QUAD4 back).
  The frozen-pose guard (`MDC_MOCAP_FROZEN_S`, built 11 Nov) stays OFF (Wesley 11 Nov: "I will disarm in this situation
  anyway it does not need to be automatic").
- Before a run: `results/registry.csv` and `docs/decisions.md`. After: one registry row per
  run, then the report. Append registry rows with LF; never rewrite the file (csv.writer
  changes every line's ending and quoting).
- Close OPEN lines in `docs/decisions.md` the session they are decided, with the run ids.
- Add files to commits by explicit path. Only `results/registry.csv` and `results/analysis/` need `-f` (12 Oct: `docs/`,
  `configs/` and `tools/` are no longer git-ignored).

## Locked decisions (do not "fix")
- No geofence disarm; one position limit (Wesley 22 Oct; 1.75 -> 1.8 m Wesley 12 Nov): any quad's x or y beyond +-1.8 m (mocap/world origin) makes the
  fleet LAND (not a disarm), and nothing is commanded or landed outside it. `pos_limit_m` 1.8 in real.yaml (on for
  every rig flight; Wesley 22 Oct, after its critic and the LIM runs); off in the twin. Rig paths are 0.75 m (real.yaml). Fleet-wide disarm on any envelope fault, except: `drop_on_loss` is on in every
  mode, attach included (Wesley 14 Oct after RIG-1014-t5_2, critic waived; detach since 8 Oct): a
  loss leaving fewer than `min_survivors`, or ring tilt > 30 deg, drops the ring (every magnet OFF,
  a welded QUAD4's coil too) and lands every drone; and an unwelded QUAD4's fault lands the fleet
  with `newcomer_fault_land`. `drop_gap_deg` 200 (Wesley 21 Oct; default off, typed on the rig only through
  `configs/rig/chain_pair*.yaml`, with `min_survivors` 2 and the tilt drop at 45 deg): a loss flies on while the
  survivors' largest gap is at most 200 deg (a T, or an opposite pair by its larger arc). A drop while Tejen flies QUAD4: on the rig `drop_takes_back` is on
  through `params_file:=configs/rig/drop_takes_back.yaml` (we take QUAD4 back and land it; code
  default off), and the console's his LAND asks his planner to land it straight down (Wesley 16 Oct).
- LAND lands every drone whatever it is doing; a freed drone off the ring is pushed to its own
  rest and disarmed alone. `newcomer_lifecycle`, `newcomer_fault_land` and `departed_ground_idle`
  are on by default (Wesley 9 Oct, after the LAND re-fly's reviewer); the legacy profile keeps them off.
  The rig's LAND defaults since 5 Nov (real.yaml): `land_descend_to_stall` (the reference keeps descending until each
  quad stops on the floor; the carriers had hovered 15-23 cm up and drifted while waiting for a taken-back QUAD4),
  `land_rearm_touchdown` (the stall test re-armed at ring touchdown; without it a carrier stronger than its map read a
  stall ~0.8 m up, R3757/R3758) and `carriers_down_alone` with its 15 cm band (while the LAND waits for QUAD4 the
  carriers on the landed ring are disarmed alone; it needs `land_descend_to_stall`, so a params_file that turns that off
  must type `carriers_down_alone: false` too, or the launch refuses). The twin has `land_rearm_touchdown` only (sim.yaml,
  since 17 Oct).
- Thrust: the rig and the rig twin use the identified affine map (offset 0.185 + 0.507*mass
  - 0.022*(V - 23.5), gain 35.2 above it; 30 Sep) with `kt_trim` off.
- The rig twin is the simulation (5 Oct). Old worlds live in `simulation_assets/old_worlds/`
  and fly only with `legacy:=true`; no mode needs it any more (the network mode flies the twin since 22-23 Oct).
- The model integral (`int_mode auto`) is the default everywhere (5 and 8 Oct). The plan's cables are massless in simulation and on the rig (Wesley 4 Nov ~23:00, after a flip of `plan_rod_level` on at ~22:20 and off at ~22:50): the magnet's weight hangs on the cable while the plan counts it in drone_mass, so a T leans on both; reported as a limitation. The twin's links match the hardware since 4 Nov ~23:15: a 0.002 kg cable + a 0.038 kg magnet at the plate, airframe 0.510 (`tools/make_carry_worlds.py`; each quad still 0.55 kg); every twin run before flew 0.040 + 0.035 on 0.475. Except (Wesley 6 Nov): the simulation claims WITH RETURNS (X2, X3, X3b's chains, X4, the rejoin cells) fly the earlier split, worlds `*_oldlinks.sdf` from `tools/make_oldlink_worlds.py` and configs in `configs/experiments_oldlinks/`: with the final masses a return takes ~35 s instead of ~17 s (R3907/R3908). Typed, the rod sets int_k_xy 0 unless chosen; real flights refuse it. The twin's weld gate filters its speed over 0.1 s (`weld_vel_filter_s`, sim.yaml, Wesley 4 Nov ~22:20).
- The network is yaw-only and open-loop in load pose except the bounded common-mode trim.
- Attach at 0.2 m/s.
- QUAD4 never flies over the carriers (Wesley 29 Oct: "I never want the incoming quad to fly that high"): it comes in
  between them, 0.30 m over the ring's plates, for the attach and with the object (`transit` low, the pickup's default;
  the rig sheet types the attach's 0.30 m). The rig launch refuses any planned QUAD4 body height over 2.6 m unless
  `transit:=over` or a `max_body_z` (2.6-3.0) is typed (Thursday 12 Nov's pickup at 1.2 m types 2.8, Wesley 5 Nov). In the twin the object lies in plate 9's gap, as on the rig. One exception, simulation only
  (Wesley 5 Nov): the 4.3 pickup-then-re-attach cell flies over (`transit:=over`, ~3.25 m) because on the low route his
  C++ planner stalls 0.7 m short of the drop point (R3767, R2904).
- The twin's links stay 1 m rods hinged at both ends (they swing as pendulums; the rig's metal cables are straight and
  taut in flight, RIG-1029 cable analysis). Wesley 29 Oct: first give the twin the rig's delays (command/radio, motor
  lag, mocap) so it reproduces the rig's in-flight swing; segmented cables (slack at rest, takeoff, releases) later.
- Combined M1 (11 Oct): `contact_still` is on by default (c0 from a still window). A take-back during a trajectory
  moves with the ring; until that is reviewed the rig refuses an orbit under `partner_contact` (hover only).
  The testing tree is `week8` (the code freeze of 5 Nov, bf52a8a), rig and twin.
- On the rig Tejen's controller does every QUAD4 attach, down to contact (the magnet only catches on the plate).
  Our motion-capture check alone confirms the attach; only then do we take QUAD4 over and fold it in (10 Oct).
  Our own approach (R8a/R8b/R9) is twin and thesis only. The check is co-motion (14 Oct, `proof_rules comotion`):
  contact, his small lift, then the tip moving with the plate on a taut tether.
- Plates count clockwise from the ring's +x seen from above: carriers on 3/11/7, QUAD4 on 9 (Tejen's 3). Angles that
  are typed or shown are clockwise too, angle = 30 x plate: carriers typed 90,330,210 (Wesley 14 Oct; the same argument
  names, meaning flipped; `docs/rig/plate_numbering.md`); the maths inside stays counter-clockwise.
- SPACE is an instant, unguarded DISARM in both the console (`tools/console.py`) and the RViz panel; both keep
  the flight buttons (Wesley 14 Oct).
