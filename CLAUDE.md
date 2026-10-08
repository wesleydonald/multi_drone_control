# multi_drone_control — working rules (read every session; the manual is general/CLAUDE.md)

Thesis: quadcopters attaching to / detaching from a cooperatively carried cable-suspended
ring payload, sim then rig. Deadline 27 Nov 2026. Owner: Wesley. Decisions are his.

This repo is in rapid software and controller development: rules, defaults and locked
decisions change often. Keep this file current: when a rule, default or decision changes,
update it here in the same session (a stale rule is worse than none) and commit it on the
current branch like any other file (it reaches main with the merges).

## Start of a session, and after every compaction
- Read `docs/SESSION_STATE.md` before acting: branch and SHA, what is running (task ids),
  decisions made this session, open questions, the next step. Trust it over the summary.
- Keep it current at every milestone (a batch started or scored, a decision, a commit, a
  merge). Anything durable also goes where it belongs straight away: decisions in
  `docs/decisions.md`, run results in `results/registry.csv`, preferences here or in memory.

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
- Headless (`tools/run_experiment.py`): laptop one at a time, no cap; lab PC up to ~6 slots
  only when nobody else is on it (check `who`, load, containers first). `/report` per matrix.
- **Keep the laptop idle for the first ~3 min of every run** (no tests, no analysis): a
  starved planner voided runs on 9 Oct. Never type `pgrep "gz sim"` (it trips clean_slate).
- A batch script that swaps code restores it in an `EXIT` trap. Bench waits use task
  notifications, never polling loops.
- Never run SIL or Gazebo while the rig stack is up on this laptop (same topics).
- Anything touching takeoff, the floor or contact gets a floor start; SIL is an air start.

## Critic and reviewer
- **Critic**: a new control law or estimator in the flight loop, or a change to a gate that
  can hold the fleet down. Not for constants or thresholds. Card and challenge in
  `docs/experiments/<date>_<name>.md` (challenge under "## Critic"), never chat-only.
- **Reviewer**: once per feature, on the final table, before the thesis or the rig.
- Wesley can waive either; note the waiver in the registry row.
- If a fix has not shown its effect in two runs, stop and report with the log.

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
- Before a run: `results/registry.csv` and `docs/decisions.md`. After: one registry row per
  run, then the report. Append registry rows with LF; never rewrite the file (csv.writer
  changes every line's ending and quoting).
- Close OPEN lines in `docs/decisions.md` the session they are decided, with the run ids.
- Add files to commits by explicit path (`tools/`, `configs/`, `docs/` need `-f`).

## Locked decisions (do not "fix")
- No geofence. Fleet-wide disarm on any envelope fault, except: detach runs with
  `drop_on_loss` (a second loss or ring tilt > 30 deg drops the ring and lands every drone,
  8 Oct); and an unwelded QUAD4's fault lands the fleet with `newcomer_fault_land`
  (Wesley 8 Oct, default off until the rig).
- Thrust: the rig and the rig twin use the identified affine map (offset 0.185 + 0.507*mass
  - 0.022*(V - 23.5), gain 35.2 above it; 30 Sep) with `kt_trim` off.
- The rig twin is the simulation (5 Oct). Old worlds live in `simulation_assets/old_worlds/`
  and fly only with `legacy:=true`; only the network mode still needs it.
- The model integral (`int_mode auto`) is the default everywhere (5 and 8 Oct).
- The network is yaw-only and open-loop in load pose except the bounded common-mode trim.
- Attach at 0.2 m/s.
