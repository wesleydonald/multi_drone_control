# TERM 3 STRATEGY — re-baseline and route to submission

Written 2026-09-09 (Tue), branch `term3` (created off `attach-week9`'s working tree).

**Decisions taken 2026-09-09 (Wesley):** (1) August work committed and pushed on
`term3`; (2) rig access ~2 days/week this term, Tejen likely available — hardware
attach stays in scope, H0 within ~2 weeks; (3) **N3 restated**: the attachment is
modelled to act like a ball joint after the weld, exactly like the tethered drones —
whatever matches the real rig; the rigid-weld control problem is out of scope, and the
`rod` variant (ball joints, tether-matched inertia) becomes the canonical attach
model; (4) deadline unchanged (Thu 27 Nov); the literature review is already drafted
in LaTeX outside the repo and will be added later. §5 below is retained for the
record.
Companion to `general/THESIS_PLAN.md` (v2) — this does not replace it; it re-baselines
the schedule after the Aug 14 → Sep 9 gap and sets the priority order for the remaining
**11.5 weeks to submission (Thu 27 Nov 2026)**.

---

## 1. Honest position, 2026-09-09

The plan's calendar says today is W6 with M3 (tracking architecture frozen) due Fri
12 Sep and hardware H1 already flown in W4. Reality:

| Thread | State |
|---|---|
| Infrastructure (Part 0) | **DONE and green** — gate (6 stages, ~2.5 min), SIL bench, headless runner, metrics/plots, backup. Verified green today (`--quick`). |
| Tracking (Part A) | **One step from the freeze decision.** Velocity loop wins tracking decisively (payload radius ratio 0.816 → 1.001, RMSE 0.272 → 0.215 m) but per-drone integrators tilt the load; `vel_ki=0` keeps the win. The prescribed fix — a **fleet-common-mode integrator** — is diagnosed in `docs/design/velocity_loop.md` but **not built**. |
| Detach (E5) | **Better than the plan knows.** The OCP-resize detach (`reconfig_mode:=ocp`, runtime attach-geometry parameterisation) beats the network hand-off on every steady metric: settled load tilt **~1° vs 26–48°**, survivor tracking error **~1 mm vs 60–910 mm** (R0111–R0114). Never analysed or written up. Two known bugs remain (post-LAND abort with a parked detached drone; ~18° residual tilt after 4→3 on the uneven surviving ring). |
| Attach (Part B — the contribution) | Three stacked faults found and fixed in the Aug 9 campaign (handover motor-cut, approach corridor through drone 0, wall-clock pose watchdog). The **weld-compliance hypothesis is refuted** by the rigid/rod/seg2 ladder — failure is invariant to joint mechanics; rigid is decisively worst (capsize <2 s, 5/5). The real remaining problem: the **hand-out is one-sided** — the newcomer eases in over 12 s but the incumbents' balanced-tension targets step in one tick (d0 elev 37→50° instantly, tilt 0.15→38°), and the ramp runs away at ~75%. Fix candidates identified, untouched. |
| Novelty claim N3 | **Currently unevidenced and needs a decision.** Every flying config welds through a ball joint = tension-only, which the reference papers already cover; the genuinely moment-transmitting `rigid` variant capsizes 5/5. Either restate N3 around what the rig physically does, or treat the rigid weld as the open control problem. Supervisor conversation, not a code fix. |
| Hardware (Part C) | **Not started. Zero rig runs. This is now the critical path.** |
| Version control | A month of work (~1,150 lines + all the Aug configs/worlds/tools) is **uncommitted**; the pushed branch ends 2026-08-05. `update.txt` §10–§12 holds the only write-up of the Aug 6–9 campaigns; `THESIS_PLAN.md` and `learning.txt` stop earlier. |

**The framing that has not changed:** detach is the working stepping stone (now with a
second, stronger mode), attach is the contribution and is mid-diagnosis with the
blocking faults cleared, hardware evidence is the deliverable.

---

## 2. The clock, re-baselined

11.5 weeks. Working backwards from Thu 27 Nov:

| Window | Milestone |
|---|---|
| W17 (Nov 24–27) | Final revisions, **SUBMIT** |
| W15–16 (Nov 10–21) | Full draft → supervisor review → revisions |
| **Fri 7 Nov** | **RESULTS FREEZE** (was Oct 31 — one week of slack spent, none left) |
| W12–14 (Oct 20–Nov 7) | Hardware attach window + gap-filling + demo capture |
| W9–11 (Sep 29–Oct 17) | Hardware carry/detach campaign; attach A3 matrix in sim |
| W7–8 (Sep 15–26) | Attach hand-out fix + boundary sweep; hardware H0/H1 |
| **This week (Sep 9–12)** | Checkpoint the repo, close Part A, write up detach, book the lab and Tejen |

Three structural consequences:

1. **Every hardware prerequisite moves to the front.** Booking lab days and Tejen's
   magnet drone is the longest lead item and gates H5/H6. If Tejen or the rig cannot
   be secured for late October, the fallback (fixed downward magnet on our own
   airframe, plan §15 R4) must be chosen **now**, not in W12.
2. **Writing starts now, in parallel.** The plan's rule — a chapter is written the week
   its method freezes — means ch. 5 (dissipative + architecture study) is writable
   the moment Part A freezes this week, and ch. 6 (detach) is writable within two.
3. **The descope ladder (§6) is pre-agreed** so that cutting scope in October is an
   execution step, not a crisis.

---

## 3. Priority stack

Ordered so that each item either unblocks the critical path or converts existing
unanalysed work into thesis evidence. Each carries its own reality check.

### P0 — Checkpoint and truth restoration (this week, ~1 day of work)

The repo's documented state is a month behind its actual state, and the actual state
exists on one disk with no commit.

- **Commit the August work on `term3`** in themed commits (velocity loop; OCP-resize
  detach + runtime geometry; attach handover safety; weld-variant ladder; tooling), and
  push. *(Requires Wesley's go-ahead — no commits without asking.)*
- Port `update.txt` §10–§12 conclusions into `learning.txt` (newest block on top) and
  the relevant `THESIS_PLAN.md` sections, so the plan stops contradicting the code.
- Wire `tools/make_weld_variants.py --check` into `gate.sh` stage 4 (generated worlds
  can currently go stale silently).
- Run archival **deferred**: real run directories are 2–10 MB each (the ladder day is
  152 MB), not the plan's ~0.5 MB estimate, so promoting raw August runs to the
  tracked `results_archive/` is off the table. The P2/P3 re-flights that produce the
  actual thesis figures are what get archived; until then the August numbers trace to
  `results/` + `update.txt`.

**Reality check:** full gate green after the commits; `git push` succeeds; archived runs
restorable.

### P1 — Close Part A: freeze the tracking architecture (~4 days)

The E3 decision is one build away. The common-mode integrator is small, fully
prescribed, and serves Part B (A2.1) as well.

1. Build the fleet-common-mode integrator in `velocity_loop.py` (pure class → unit
   tests), behind a parameter defaulting to current behaviour.
2. Validation ladder as established: unit → SIL 3-drone carry → 5-repeat Gazebo circle,
   three arms: MPC baseline, `vel_ki=0`, common-mode. One launch arg apart, medians +
   ranges, stage-split diagnostic on every arm.
3. **Freeze by Fri 19 Sep** on the plan's gate rule: the winner must beat MPC tracking
   without giving up load attitude (settled tilt ≤ 10°). Ties → MPC stays default and
   E3 is reported as the measured architecture study either way.
4. Write the ch. 5 methods/results section that week.

**Reality check:** the decision uses only Gazebo numbers (SIL is necessary, not
sufficient); a 1-in-5 lift failure rate on this config is already documented, so 5
repeats with an abort-exclusion rule stated up front.

### P2 — Convert detach into a finished chapter (~3 days, high value/cost ratio)

The OCP-resize result is the strongest unclaimed evidence in the repo.

1. Fix the post-LAND abort (LAND path must tolerate a parked detached drone) — it is
   the only thing between R0113/R0114 and clean `passed: True` runs.
2. Fix the two run-wasters found in August, which tax every future batch: the ARM race
   to `approach_mpc_3` (largest single source of wasted attach runs) and pre-takeoff
   disarms not triggering fleet abort (produces junk data that looks like flight).
3. Re-fly the one-flag A/B at 5 repeats per arm; extend to 3→2 and hover + circle
   (the E5 matrix). `compare_runs` + write-up same day.
4. Investigate the ~18° residual tilt after 4→3 with `metrics.cog_margin` (already
   built, never reported) — wire it into `summarise_run` so the support-polygon
   argument appears in every run's metrics. It may be a *geometry* result (180° gap =
   boundary of the support polygon), which is a figure, not a bug.

**Reality check:** every number from `tools/metrics.py`; params read-back checked on
both arms; runs archived on the day the figures are made.

### P3 — Attach ring: fix the hand-out, then characterise the boundary (W7–W9)

The blocking faults are gone; the weld-mechanics question is answered; what remains is
exactly the reference-side problem the user steered toward (fix geometry/references,
not the tracker).

1. **Symmetric hand-out** — ease the *incumbents'* balanced-tension solve and slot
   targets over the same ramp as the newcomer (blend the wrench-solve's target set by
   the hand-out scalar), so no drone's reference steps. Reference-side only, behind the
   existing `attach_handout` machinery, offline-gated, then Gazebo.
2. If the ~75%-ramp runaway persists: slow `T_handout` (12 → 20, 30), then the
   elevation ladder. These are one-arg experiment configs that already exist in
   pattern.
3. **Elevation sweep {45, 55, 65} × 5 repeats → the stability-boundary curve (N4)** —
   a thesis figure regardless of where the boundary lands. The plan's A3 success bar
   stands unchanged: settled tilt ≤ 10°, height within 0.05 m, newcomer ≥ 15% tension,
   ≥ 60 s sustained, trajectory resumed.
4. **N3 decided (2026-09-09):** the sim attachment must behave like the real rig — a
   ball joint after the weld, the newcomer indistinguishable from the tethered drones.
   The `rod` variant is canonical; retire `rigid`/`seg2` to the ladder's evidence
   role. A3 demonstrates reconfiguration under that model, and the thesis states the
   tension-only nature of the weld plainly.
5. Fallback ladder unchanged (plan §12.2): modest-tilt ring member → 2+1 geometry →
   central lifter as headline with the boundary curve as the characterised negative.

**Reality check:** offline A–K is a smoke test here, not evidence — the PD plant
settles cases Gazebo capsizes. Ring-attach claims come only from Gazebo (then
hardware). Watch `|aCm|` vs `|aC|` and ERR per the established method.

### P4 — Hardware campaign (starts as soon as the rig is booked; critical path)

Compressed from the plan's H0–H8, preserving the order that de-risks:

| Stage | Content | Earliest |
|---|---|---|
| H0 | Bring-up: mocap routing, ELRS, per-airframe kT, solo hover, abort test | W7 |
| H1 | 2-drone carry — **establishes the sim-to-real gap; everything recalibrates on this** | W7–8 |
| H2/H3 | 3–4 drone carry + **detach** (both modes if H1 is clean) | W9–10 |
| H4 | Frozen dissipative/velocity architecture across trajectories | W10 |
| H5/H6 | Magnet approach (no weld) → hover weld → moving weld — **needs Tejen booked now** | W11–13 |

Standing rules unchanged: no first-time-anywhere code at the rig; flight card per
session; same-day analysis + backup; every hardware config has a sim twin (E9 free).
The 2026-08-03 real-world yaw fixes are already in; `real_control_launch.py` now
exposes `attach_radius`/`attach_z`, so the hardware side of the attach geometry is
describable.

**Minimum publishable hardware result:** carry + detach on the rig (H0–H3) with the
sim-paired gap study. Hardware attach is the stretch goal, not the floor.

---

## 4. Working method — how the agent iterates fast without lying to itself

The verification ladder, cheapest first, with what each rung can and cannot prove:

| Rung | Cost | Proves | Cannot prove |
|---|---|---|---|
| pytest + import check (gate 1–3) | seconds | logic, wiring | anything dynamic |
| `verify_dissipative` A–K (gate 5) | ~1 min | network correctness, detach/attach mechanics offline | Gazebo divergences (PD proxy, no MPC dynamics) |
| SIL bench | minutes | real nodes over real topics, mechanisms (e.g. transit error) | plant-coupled runaways it has not reproduced |
| Headless Gazebo (`run_experiment.py`) | ~4 min/run | the actual claim, 5 repeats | hardware effects |
| Rig | 1–2 days/wk | the thesis | — |

Rules already earned by this project, restated as the operating contract:

1. **Never promote a claim above the rung that produced it.** "Settles offline" is
   reported as exactly that.
2. Full gate before any Gazebo batch and before any commit; say plainly when the gate
   does not cover the change.
3. One variable per A/B; configs assert the single-arg difference; params proven by
   read-back, not launch args; sweep-window metrics only; medians + full range, n=5
   sim (with a pre-stated abort-exclusion rule).
4. Every finding — especially negatives (T1, the weld ladder) — lands in
   `learning.txt` the same day. The August gap between code and documentation is the
   anti-pattern this rule exists for.
5. `tools/clean_slate.sh` between runs; RViz launch first; `/dev/shm` cleared.
6. New behaviour behind a default-off parameter; wire format frozen; tracker MPC
   weights untouched.
7. Anything behind a figure is archived and pushed the same day.

---

## 5. Decisions needed from Wesley (none block P0–P2 start)

1. **Commit approval** — may I make the P0 checkpoint commits on `term3` and push?
2. **Lab logistics** — how many rig days/week this term, from when, and Tejen's
   availability window? (Determines whether hardware attach survives or the R4
   fallback triggers.)
3. **N3 positioning** — restate the novelty around the physical system's actual joint
   chain (rendezvous + weld + handover + reconfiguration, with the weld's compliance
   characterised), or commit to solving control under a genuinely rigid weld? The
   ladder data says the second is a real open control problem, not a tuning exercise.
   Recommend raising with the supervisor this week.
4. **Deadline confirmation** — still Thu 27 Nov? Any supervisor feedback from the gap
   that changes priorities?
5. **Thesis text** — does any chapter draft exist yet? If not, the background/novelty
   positioning section (plan §1.2) is written this week alongside P1.

---

## 6. Descope ladder (pre-agreed cuts, in order, if the clock wins)

1. E10-F failure-redistribution study (sim-only extra) — cite the mechanism, drop the runs.
2. E8 scalability beyond spot checks at n=2/4.
3. fig-8 everywhere → circle + line only.
4. E10 robustness matrix → one representative perturbation per axis.
5. Hardware attach: moving weld → hover weld → approach-without-weld (H5 only) →
   sim-only attach with the hardware campaign ending at detach. Each step down is
   reported as a limitation, not hidden.

The floor that is not descoped: hardware carry + detach, the sim attach boundary
curve, the architecture study, and honest sim-to-real accounting.
