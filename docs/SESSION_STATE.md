# Session state (the LAB chat reads this first, via `/lab`; keep it short and current)

Updated: Fri 30 Oct 2026, 17:18 AEDT (from `date`). Branch **week8** at the 30 Oct 17:30 commit (see `git log -1`), nothing pushed (Wesley: push week8
once the T fix is merged). The full log up to this point: `docs/archive/session_state_to_2026-10-30.md`. History also in
`docs/decisions.md`, `results/registry.csv` and `git log`.

## Running when the last chat closed (30 Oct 17:30)
- **Lab PC, overnight, detached from any chat:** `~/mdc_overnight/lab_overnight_1030.sh` (its own session; logs
  `~/mdc_overnight/lab_x3b.log`, `~/mdc_overnight/lab_x4.log`). Flying the rest of
  X3b from R3010 (15 runs: s9 hover and chain at the real-time setting, the it5 comparators, then every config's second repeat),
  then X4 (6 runs: the short relay, the two 30-min relays, the fixed team x2, the 1.15 kg relay). Nobody scores them
  yet: score X3b on its card (`docs/experiments/2026-10-30_x3b_realtime_8_9.md`) and X4 with `tools/x4_metrics.py`, and
  write their registry rows. To stop it: kill the script's PID (`pgrep -af lab_overnight_1030`), then SIGINT its
  lab_batch (it removes its containers). Check `ssh drones@drones docker ps` after any stop: a run that starts as the
  batch is stopped can leave its container up (30 Oct 17:15 two did, and the next batch refused to start;
  `docker rm -f wesley_slotN; docker network rm wesley_slotN` on the lab PC clears them). Relaunched 17:18 after that.
- Nothing else: no agents, no laptop Gazebo.

## First job: the T fix (Wesley 30 Oct: "it shows our method is worse than doing nothing, this needs to be fixed")
Before the code freeze (Fri 6 Nov). X2-full and X8 both wait for it.
- **The defect:** after an unannounced release turns the square into a T, our planner settled the ring 7-13 deg off
  level (X1 sqT R2965/R2972; today's tree R2997 6.95), where flying on without the resize settled at 2.2.
- **The cause** (four agents, `docs/analysis/t_offset_{model,forensics,attach}_2026-10-30.md`): the T's middle carrier
  (plate 3) has a level share of zero, so the planner holds it at its 0.1 N tautness floor (`planner_ocp.py:63`,
  `geometry.py:242`); the sideways model integral (`int_k_xy` 5.26, the default) pulls the ring back to its point
  against the twin's unmodelled rod weight; the rigid rod then lifts plate 3 and the ring rolls about the pair's line.
  Joins and detaches into the same formation behave the same (attach answer: no difference by direction).
- **Results so far** (exploratory, settled tilt in deg, T6 in s):

| Run | Variant | Settled | Notes |
|---|---|---|---|
| R2965 / R2972, R2997 | ours (baseline) | 8.30 / 10.52, 6.95 | T6 5.4 / 13.4 |
| R2966 / R2973 | no resize | 2.19 / 2.19 | |
| R2998, R3202 | ours, `int_k_xy` 0 | 0.58, 0.60 | T6 5.8; ring ~6 cm off its point |
| R2999, R3203 | no resize, `int_k_xy` 0 | 2.19, 2.15 | plate 3 down |
| R3000 | ours, rod mass modelled | 5.19 | dip 115 mm |
| R3201 (R3204) | ours + `plan_zero_share_floor` | 1.06 | T6 14.5: a slow -4 deg swing while the integral builds |
| R3002 | pair -> T rejoin, `int_k_xy` 5.26 (a JOIN into a T) | 6.91 | the join tilts like the release |
| R3003 | pair -> T rejoin, `int_k_xy` 0 | 0.83 | |
| R3204 | ours + `plan_zero_share_floor` (in flight when repeats stopped) | 1.04 | T6 14.4 |
| R3205 | the even cell (x1_h4) + the switch | 0.05 | T6 3.9: unchanged, the switch never fires |

Unflown (one run each, A's card "Next"): the figure-8 T with the switch (`x1_fT_zsf`, baseline `x1_fT_ours`), the
join into a T with the switch (`rj3_..._kxy_zsf`), the switch with `int_k_xy` 0 (`x1_sqTnoxy_zsf`).

- **A's verdict** (its card): the switch levels the T (1.04/1.06 deg, on its point) but settles slowly (T6 14.4-14.5 s:
  a -4 deg swing while the integral builds; at 0 N the plan can only pull plate 3 up); the dip is borderline (R3204
  106.6 mm vs ours' max 100.2); the even cell is unchanged. **Wesley decides:** (1) the claim arm: the zero floor (level,
  on its point, slow), `int_k_xy` 0 for T formations (0.6 deg in 6 s, 64 mm off its point, what the rig already
  flies), or both (unflown); A recommends the switch, reporting the slow settle; (2) U5's T6 bar: keep 5 s or accept
  ~15 s for a level T; (3) the switch stays refused on the rig (near-slack middle cable) until the critic.
- Before the merge: `gate.sh --quick` on A's worktree (not run: the laptop was kept quiet); bringup's test_flake8 and
  test_pep257 fail there (long lines `real_mode.py:179,191` already in 5a7ccd0, plus the new launch test's docstrings):
  check against 5a7ccd0 and fix. A's `x1_*_zsf` / `x1_fT_*` configs point at main-checkout `configs/twin` files: copy
  any that exist only in the worktree. A's runs R3201-R3205 are copied into the main results/ and registry (30 Oct).
- **The fix code:** agent A's worktree `.claude/worktrees/agent-a4a6813c25d6d67f3` (branch `worktree-agent-a4a6813c25d6d67f3`, based on 5a7ccd0), switch `plan_zero_share_floor` (twin only,
  default off), card `docs/experiments/2026-10-30_t_settle_fix.md` there (PROPOSED bars, empty "## Critic"). 
- **Next, in order:** (1) the critic on A's card; (2) Wesley picks the default: `int_k_xy` 0 in uneven formations (a
  launch default; already how every real-world chain file flies), the zero floor (a planner constraint), or both;
  (3) merge behind the switch, gate, commit; (4) one run per still-unflown variant (A's card, "Next"); (5) the claim
  runs, then X1's sqT cell re-flown, then X2-full and X8; (6) QUAD4's fold-in gets the rejoin's weld-height cap (card
  and critic; Wesley 30 Oct; R2829 peaked 9.9 deg).

## Then (the direction at the top of `docs/handin_plan.md`)
- Real world S1 Wed 4 Nov: `docs/rig/wed_2026-11-04.md`, every line desk-checked; QUAD4's W and L measured on the day.
- S2 Thu 5 Nov sheet Mon/Tue: 2 flights per 1 m baseline cell; the thrust-margin check from the logs first.
- X3b's two timing runs (`*_rt0`) and the solver bench on this laptop on a quiet day; then the solve-time figure.
- Code freeze Fri 6 Nov; X11 (`docs/analysis/x11_refly_manifest.md`, `configs/batches/x11_A.txt`) and every claim on
  the final tree.

## Owed
- Registry rows: R2997-R3003 written (exploratory), R3004-R3009 VOID (cancelled repeats); X3b R2979-R2983 written (3 flown
  unscored, 2 VOID). Owed: the overnight X3b/X4 runs, and A's R3201+ (in its worktree registry).
- The results feed (`docs/results_feed.md`): post the T fix's result for the thesis chat when it is final.
- Tejen follow-ups (`docs/handin_plan.md` section 6).
- Cleanup for Wesley to run: X4's old worktree is merged (30 Oct 16:12): `git worktree remove --force
  .claude/worktrees/agent-a6ad05d91f3a03b90 && git branch -D worktree-agent-a6ad05d91f3a03b90`. A's T-fix worktree
  (`agent-a4a6813c25d6d67f3`) stays until its fix is merged.

## How to work (Wesley 30 Oct)
- Exploratory runs once per variant, no repeats; repeats only for claims under frozen bars.
- No agent cap for now (Wesley 30 Oct); spawn what the work needs.
- Commits on Wesley's word only, never push; he pushes week8 once the T fix is merged.
