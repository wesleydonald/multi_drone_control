# Card (DRAFT for Wesley, 22 Oct): chain step 2a, the force-vector tension split with the tether's weight

Draft written overnight from the chain card's results (2026-10-21_chain_release) for review after the rig day.
Nothing is built. The critic is required before any code (a change to the reference the planner flies).

## 1. Question
Can the planner hold the 90 deg T (plates 0/3/6, the quad opposite the gap carrying ~nothing) level? It hangs
6.8 deg today and 4.4 deg with the tether-mass model.

## 2. Hypothesis
- The T's tilt comes from the split, not the controller.
- **Today:** the reference balance solves tensions along FIXED cable directions (60 deg up, radially out). The
  rider's cable can pull up on plate 3 only by also pulling it outward, which nothing balances. The least-squares
  split therefore gives it the 0.1 N floor (massless model) or 0.43 N (tether mass, R2258), against the ~0.7 N its
  tether weighs on the plate.
- **The change:** solve for force VECTORS, each within a small cone of its nominal direction (the quads move to make
  them). The plan's exploration found 5.73 / 5.73 / 0.70 N, level, with quads 0 and 6 leaning ~1 deg toward plate 3.
- **Prediction:** the T settles under 2 deg (from 4.4 / 6.8). The pair after the release starts level, so its
  rocking falls from 6.6-8.4 deg p-p.

## 3. The one variable
`split_mode: vector` (new switch, default `fixed`):
- `geometry.balanced_tensions` gains a vector solve;
- `ReferenceBuilder` takes its directions and tensions as `s_nom` / `t_nom`.

Everything else as `t3_pair_release_rodmass` (R2258): tether-mass model on (`rod_mass` 0.075, `rod_com` 0.147, own
solver cache), `chain_pair.yaml`. Fix on the way, behind the same switch: `dissipative_node.py:2693` re-solves the
split WITHOUT the rod terms after a resize.

## 4. Baseline
| Run | T tilt before the release | Rider planned | Pair roll p-p, last 10 s |
|---|---|---|---|
| R2258 (tether mass, fixed directions) | 4.4 deg | 0.43 N | 6.6 deg |
| R2250 / R2251 (massless) | 6.8 / 6.8 deg | 0.13 N | 8.4 / 7.8 deg |

## 5. Pass / fail numbers (to freeze with Wesley before any run)
- **Confirms:**
  - T settled tilt (last 10 s before the release) under 2 deg in both runs;
  - the rider planned 0.6-0.8 N;
  - no drop or abort;
  - the even 3-quad hover (`b3_hover`) within 0.3 deg of R2256 (the split must leave even rings alone).
- **Falsifies:**
  - T settled 3 deg or more in either run;
  - a drop or abort;
  - the even hover moved by more than 0.5 deg.
- **Informational:** the pair's rocking after the release.

## 6. Repeats
2 per arm, plus neighbours once: the square-to-T release, the T on a 1 m figure-8, and the even 3-quad hover.

## 7. Cost
~8 laptop runs (~45 min).

## 8. New code?
Yes:
- `geometry.vector_balanced_tensions`;
- the `split_mode` switch through `ReferenceBuilder`, `planner_node`, `common.yaml` and `real_mode.PLANNER_OPTIONAL`;
- the resize fix;
- tests: the T, the even ring unchanged, the 4-quad uneven ring 30/90/150/270.

Critic before the runs. If it fails, the switch stays `fixed` and the code is removed.

**Open for Wesley:**
- **Is the 4.4 to 6.8 deg T worth fixing before the rig chain, against the transient?** The square-to-T release
  rolls the ring to 57 deg (R2249), which the drop rule turns into a drop at 45 deg (R2254). Tonight's
  marker-check comparison shows how much a faster detection helps that transient. The rest of the transient is the
  re-plan plus the trackers' 25 m/s^3 pull slew: candidate step 2b.
- **Twin and rig differ:** the twin's rigid rods carry their weight differently from the rig's slack cables. The rig
  rider's tether is lighter (a cable plus the magnet). So the size of the gain on the rig is unknown until the rig T
  flies (Part C, today).
