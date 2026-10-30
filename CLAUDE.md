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
- Shared: `docs/handin_plan.md` (each chat ticks its own items; the lab chat ticks a result with its run ids and card
  when it is final, which is the thesis chat's signal to write it up), `docs/decisions.md` (append only, both),
  `docs/sim_runs_todo.md` (the lab chat). A one-off question can go to either chat if it changes nothing of the other's.

## Start of a session, and after every compaction
- Read your chat's state file before acting (`docs/SESSION_STATE.md`, or `docs/THESIS_STATE.md` in the thesis chat): branch and SHA, what is running (task ids),
  decisions made this session, open questions, the next step. Trust it over the summary.
- Keep it current at every milestone (a batch started or scored, a decision, a commit, a
  merge). Anything durable also goes where it belongs straight away: decisions in
  `docs/decisions.md`, run results in `results/registry.csv`, preferences here or in memory.
- 22 Oct ultracode push: its authority, decided values and ordered work queue are at the top of
  `docs/SESSION_STATE.md`; the designs and the critic's integration order are in `docs/analysis/push_1022_plan.md`.
- Rescope of 30 Oct (`docs/plan_rescope_2026-10-30.md`, approved except X9, decisions.md): headline X1 (release ablation)
  and X4 (8-quad endurance relay); freeze by Thu 19 Nov at the latest, aim Thu 12 Nov; hand-in Fri 20 Nov.
- Until hand-in: `docs/handin_plan.md` (everything left: writing, supervisor, lab sessions, figures, reviews) and
  `docs/sim_runs_todo.md` (every simulation run left). Tick items the moment they close.
- The current plan is `docs/plan_thesis_results_2026-10-21.md` (approved 21 Oct: the supervisor's results spec,
  1 m cables, the 90 deg chain, rig 4->3->4 with the basket, sim U/S/C, videos). Its status table is how Wesley
  follows progress: update the row (status, commits, run ids) the moment a work package moves.

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
- **Claim** (a thesis table or a rig go/no-go): experiment card (`/experiment`), falsifier
  and bars written before the runs, numbers in a table against the named baseline, run ids.
- **Bars are frozen before the runs.** If a bar turns out wrong, score against the original
  and use the new one only for later runs; never re-score runs already seen.
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
  real.yaml; Wesley 29 Oct): the 1 m ring bends at 5.3 Hz (RIG-1029-h4); two 2 Hz low-pass stages stopped it but their lag
  let the triangle swing at 1.1 Hz (RIG-1029-sq2_2/-sq2_4), so the low-pass is off (`cable_lp_hz` 0). common.yaml: both
  off (the twin's ring is rigid).
- The payload is the 1.15 kg ring of 28 Oct (real.yaml: load_mass, inertia estimate, attach_radius 0.204, attach_z
  0.025 estimate, payload_rest_z 0.013; the planner solver is built at its mass). The 0.86 kg M2A ring flies with its
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
- `python3 tools/prebuild_planner.py` after touching the planner or masses, before any batch.
- Rig day, before T1 (stack down): `python3 tools/mocap_stream_check.py --expect <bodies>` must say OK. Motive streams
  only the bodies the flight uses, at most 8 at 240 Hz (attach 6,8,11-14,24; 3 quads 8,11-13,21-23): the mocap PC's
  forwarder tops out at ~2,250 packets/s, and on 30 Oct 10 bodies made every pose 95 ms late and every ring lift heaved
  (RIG-1030-bench_mocap).
- Before a run: `results/registry.csv` and `docs/decisions.md`. After: one registry row per
  run, then the report. Append registry rows with LF; never rewrite the file (csv.writer
  changes every line's ending and quoting).
- Close OPEN lines in `docs/decisions.md` the session they are decided, with the run ids.
- Add files to commits by explicit path. Only `results/registry.csv` and `results/analysis/` need `-f` (12 Oct: `docs/`,
  `configs/` and `tools/` are no longer git-ignored).

## Locked decisions (do not "fix")
- No geofence disarm; one position limit (Wesley 22 Oct): any quad's x or y beyond +-1.75 m (mocap/world origin) makes the
  fleet LAND (not a disarm), and nothing is commanded or landed outside it. `pos_limit_m` 1.75 in real.yaml (on for
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
  `land_rearm_touchdown` (re-arm the LAND stall test at ring touchdown) is on in the twin (sim.yaml; Wesley 17 Oct);
  the rig gets it after a critic look at its wall-clock pose check.
- Thrust: the rig and the rig twin use the identified affine map (offset 0.185 + 0.507*mass
  - 0.022*(V - 23.5), gain 35.2 above it; 30 Sep) with `kt_trim` off.
- The rig twin is the simulation (5 Oct). Old worlds live in `simulation_assets/old_worlds/`
  and fly only with `legacy:=true`; only the network mode still needs it.
- The model integral (`int_mode auto`) is the default everywhere (5 and 8 Oct).
- The network is yaw-only and open-loop in load pose except the bounded common-mode trim.
- Attach at 0.2 m/s.
- QUAD4 never flies over the carriers (Wesley 29 Oct: "I never want the incoming quad to fly that high"): it comes in
  between them, 0.30 m over the ring's plates, for the attach and with the object (`transit` low, the pickup's default;
  the rig sheet types the attach's 0.30 m). The rig launch refuses any planned QUAD4 body height over 2.6 m unless
  `transit:=over` is typed. In the twin the object lies in plate 9's gap, as on the rig.
- The twin's links stay 1 m rods hinged at both ends (they swing as pendulums; the rig's metal cables are straight and
  taut in flight, RIG-1029 cable analysis). Wesley 29 Oct: first give the twin the rig's delays (command/radio, motor
  lag, mocap) so it reproduces the rig's in-flight swing; segmented cables (slack at rest, takeoff, releases) later.
- Combined M1 (11 Oct): `contact_still` is on by default (c0 from a still window). A take-back during a trajectory
  moves with the ring; until that is reviewed the rig refuses an orbit under `partner_contact` (hover only).
  week4 stays as it is. `week5` (12 Oct: rig-wed + the take-back race fix) is the testing tree from Wed 14, rig and twin.
- On the rig Tejen's controller does every QUAD4 attach, down to contact (the magnet only catches on the plate).
  Our motion-capture check alone confirms the attach; only then do we take QUAD4 over and fold it in (10 Oct).
  Our own approach (R8a/R8b/R9) is twin and thesis only. The check is co-motion (14 Oct, `proof_rules comotion`):
  contact, his small lift, then the tip moving with the plate on a taut tether.
- Plates count clockwise from the ring's +x seen from above: carriers on 3/11/7, QUAD4 on 9 (Tejen's 3). Angles that
  are typed or shown are clockwise too, angle = 30 x plate: carriers typed 90,330,210 (Wesley 14 Oct; the same argument
  names, meaning flipped; `docs/rig/plate_numbering.md`); the maths inside stays counter-clockwise.
- SPACE is an instant, unguarded DISARM in both the console (`tools/console.py`) and the RViz panel; both keep
  the flight buttons (Wesley 14 Oct).
