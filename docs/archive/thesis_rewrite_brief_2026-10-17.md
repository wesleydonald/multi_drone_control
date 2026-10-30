# Brief: thesis rewrite, full version then a condensed one (Wesley 17 Oct evening)

For a separate session. Recommended: **Opus 5.5, effort high** (xhigh for the condensing pass if it is cheap enough).
It is long, careful editing against evidence, not search.

Paste this to start it:

> Read `~/multi_drone_control/docs/thesis_rewrite_brief_2026-10-17.md` and do what it says. Work in the thesis
> worktree only. Ask me batched questions where the brief says to; otherwise take the best option and note it.

## What Wesley asked for

- Condense the thesis to what is important. Decide what moves to the appendix. Start to look at the 50-page limit.
- **First the best version with no page limit, then a condensed and refined one, so he can see both.**

## Where things are

- **The thesis:** worktree `~/mdc_wt/thesis` (branch `thesis-rig`), `docs/thesis_drafts/thesis_v2/`.
  - `main.tex` inputs `Source/*.tex`, which input `Source/src/*.tex` and `Source/src/tables/*.tex`.
  - It builds clean at 122 pp, about 58k words.
  - The largest files: `Appendices.tex` 8.1k words, `discussion.tex` 4.3k, `res_steady.tex` 3.2k,
    `reconfiguration.tex` 3.2k, `sim_detach.tex` 2.8k, `controller.tex` 2.6k, `res_carry.tex` 2.5k,
    `Project_plan.tex` 2.1k.
  - The `skel_*.tex` files are Chapter 4 skeletons (carry, leave, join, ledger, discussion).
- **The evidence:** `~/multi_drone_control` (branch `day-1016`, Wesley merges into `week5`). This is read-only for
  you: an overnight twin batch is flying from it, and its install links to `src/`.
  - `docs/analysis/sim_results_inventory_2026-10-16.md`: **the master map.** Every result, how current it is, what it
    supports, the figure and table plan, and Wesley's decisions on it.
  - `docs/analysis/appendix_refly_2026-10-17.md`: the appendix tables re-flown on the freeze stack. 54/54 pass. Some
    claims are falsified (below).
  - Cards with frozen bars and reviewer verdicts, in `docs/experiments/`:
    - carry: `2026-10-16_carry_speed_envelope.md`;
    - detach: `2026-10-12_tier_a_crossover.md`, `2026-10-16_tier_b_support_margin.md`, the W4/W5 freeze
      `docs/rig/freeze_2026-10-14.md`;
    - corner cutting: `2026-10-16_corner_cutting.md`;
    - LAND: `2026-10-17_touchdown_height_gate.md`, `2026-10-17_land_ref_below_rest.md`;
    - rig attach: `2026-10-15_attach_comotion_rig.md`.
  - `docs/decisions.md`: the latest DECIDED lines win. `results/registry.csv` holds one row per run.
  - The morning reports in `docs/reports/`, and `docs/reports/supervisor_brief_2026-10-16.md`.

## Rules (from the working rules; they apply to the thesis)

- **No run ids, flight names, SHAs or branch names** in the thesis text.
- **No "dissipative" wording.** Say "our planner" (mpc_planner) and "detach mode". The network controller is legacy:
  appendix or cut.
- **Plates count clockwise from the ring's +x**, angle = 30 x plate. Carriers are on 3/11/7, QUAD4 on 9. Check every
  figure and caption.
- **Numbers go in tables, not prose.** A claim the table makes is stated once, with its reviewer caveats. Each card has
  a "must not say" list: honour it.
- **Only frozen, reviewed results are claims.** Exploratory runs may be mentioned as exploratory, or not at all.
- **Figures follow the `thesis-figure` skill** (`~/multi_drone_control/.claude/skills/thesis-figure`): journal
  treatment, the RViz palette kept.
- **Keep blank-line paragraph breaks.**
- **Never mention Claude or AI.** Commits are authored by Wesley, with no co-author or attribution trailers. Never push.
  Commit on `thesis-rig` by explicit path.
- **No Gazebo, SIL or test suites:** the laptop is flying a twin batch overnight. Build LaTeX with `nice -n 19 latexmk`,
  and not more often than needed.

## What has changed since the draft (fix these first)

1. **Vertical claims (appendix C-D2, C-D3, C-D5) are falsified on the freeze stack.** The measured pull (the default
   `cable_source mocap`) absorbs a vertical push and a mass error within about 1 s; the integral is not what removes
   them. With the planned pull (`cable_source model`), the push leaves -7.95 cm and +20 % mass leaves +22.2 cm.
   - Rewrite the disturbance and robustness text around the measured pull.
   - The radius +2 cm row's "learned force" (0.76 N) is wind-up, not learned: do not present it as learned.
   - C-L1 is not supported (the false touchdown, item 4).
2. **Carry:** the three-carrier claim on every shape at three speeds. 39/40 pass. Spin at 0.3 m/s is outside the tilt
   envelope. The 1 m cables carry as well as the 0.55 m rods.
   - The frozen path RMSE scores against the 10 Hz held reference, so an on-time ring pays 17 mm at 0.3 m/s. Report
     the clock-free cut and across-path error, and the lag in seconds (Wesley 16 Oct).
   - Corner cutting is closed (17 Oct): the defaults stay, the lag is reported. The timing fix is exploratory; it fails
     on spin. At most one appendix paragraph.
3. **Detach:** the freeze cards (Tier A T3/T8/T9/K1/T15, Tier B, W4/W5, drop and land D1/D4) are scored, with the
   reviewer's corrections. These are the Chapter 4 detach tables.
4. **LAND (17 Oct):** with the old defaults the stall touchdown test can disarm the fleet about 0.4 m up after the ring
   lands (twin, the rig's LAND path: 2 of 4). Re-arming the test at ring touchdown fixes it (0 of 5) and is the twin
   default now; the rig gets it after the critic's fixes. A drone stronger than its thrust map can still disarm high;
   a fix is under test. This is at most one paragraph plus a limitation; the landing tables keep their freeze tree.
5. **Join (attach, T11/T12):** these fly on a second tree after the 21-22 Oct rig day. Keep the placeholders, sized
   honestly. Tonight's exploratory join re-fly is not thesis evidence.
6. **Stale or legacy rows** (inventory section 4, ranks 11-12: C1, D6, C4, A8, C5, C9, D10, F3; and D9, the network):
   cut them, or move them to the appendix as dated history, one line each.

## How to work

1. **Map first** (`thesis_v2/REWRITE_MAP.md`): one row per section, with its words now, what it claims, whether
   the claim is current (inventory, cards, decisions), and the action: keep / rewrite / condense / to appendix / cut.
   Note where text repeats (literature in several places, the controller described twice, the Project_plan and
   Project_preparations chapters).
2. **Version A, no page limit:** copy `thesis_v2` to `thesis_v3_full`. Fix every item above, merge the skeletons into
   real Chapter 4 sections, cut repetition, and keep everything that earns its place. It must build.
3. **Version B, condensed toward 50 pages:** copy `thesis_v3_full` to `thesis_v3_50`.
   - Count the main matter (introduction through conclusion) against the 50 pages, unless the faculty rules in the
     repo or the title page say otherwise; report both counts.
   - Move derivations, parameter tables, secondary results and history to the appendix. Keep the narrative, the claims
     and one figure per claim.
   - It must build.
4. **For Wesley** (`thesis_v3_50/CHANGES.md`):
   - the page counts of v2, v3_full and v3_50 (main matter and total);
   - what moved, what was cut and why, one line each;
   - what is still a placeholder (join, the rig rows);
   - his decisions, batched with a recommendation first. For example: is the 50 the main matter or the whole PDF? Do
     the project-plan and preparations chapters stay? How much of the literature review?
5. Commit both versions on `thesis-rig` (Wesley only, no trailers). Do not delete `thesis_v2`.

## What not to do

- Do not invent or re-derive numbers. Take them from the cards, the inventory and the registry. Where they disagree,
  the latest reviewed card wins; note the conflict.
- Do not touch `~/multi_drone_control`: no writes, no git there.
- Do not fly anything.
