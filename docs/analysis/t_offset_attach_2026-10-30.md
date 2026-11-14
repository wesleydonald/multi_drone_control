# Does the resize leave the ring worse after a JOIN too? (attach side, 30 Oct 2026)

Wesley, 30 Oct: "If doing nothing is better than resizing we have some serious issues, does this happen for both
detaching and attaching?" The detach finding: X1's sqT cell (R2965-R2978, card
`docs/experiments/2026-10-30_x1_release_ablation.md`): after an unannounced release turns the square into a T (0/3/6),
our planner's resize settles the ring 8.3 / 10.5 deg off level, and flying on without the resize settles it at 2.2 deg.
This note is the attach side: every flown join, read from run.csv, events.csv and the planner logs. Read-only: no code
or config was changed. Three configs were written for the lab chat to queue (section 5).

Plots: `results/analysis/t_offset_attach_2026-10-30/` (fig1: the 8-quad chain, the same formations reached both ways;
fig2: settled tilt by formation, direction and xy integral gain; fig3: tilt through the fold-in, every kind of join).
Scripts (scratchpad, not committed): `join_tilt.py` (per weld), `phases.py` (per formation phase), `plots.py`.

## Verdict

**No: it does not depend on the direction.** A join ends at the same settled tilt as a detach into the same formation
(the 8-quad chain flies every formation both ways in one flight: 1.03-1.06 vs 1.01-1.03 deg, 1.13-1.14 vs 1.14-1.16,
0.01-0.05 vs 0.02-0.04; the T with the xy integral off, 0.76-0.79 by a join vs 0.52-0.75 by a detach).

What sets the settled tilt is the formation together with the xy part of the model integral (`int_k_xy`, 5.26 N/m in
common.yaml, 0 in the chain's rig files). Every T at 5.26 hangs 6.6-10.5 deg off level, however it was reached (an
unannounced release, a soft detach, or the T flown from takeoff). Every T at 0 hangs at 0.5-0.8 deg, again however it
was reached, a join included. The X1 sqT cell flew 5.26 (`configs/twin/x1_sqT.yaml` has no `int_k_xy` line). The
unannounced square-to-T release with the resize at `int_k_xy` 0 (R2823, 27 Oct) settled at **0.60 deg**, below X1's
no-resize arm (2.19). So the detach finding is most likely "the xy integral in a T". It is not "the resize". This is a
lead for the detach agents, unconfirmed until the section 5 runs fly.

The detach agents' cause is the 0.1 N tension floor on the T's rider (section 1b). The floor is the same in both
directions, and the pair -> T joins plan their rider at 0.10-0.13 N. Those joins settle at 0.72-0.82 deg at `int_k_xy`
0, as do the T's reached by a detach at 0. So the floor tilts the T 7-10 deg only when the xy integral pins the ring's
centre.

In the narrower sense the answer is yes. A join that makes the formation uneven leaves the ring more tilted than it was
before the join, by the same amount as a detach into that formation: 4 -> 5 and 6 -> 7 of 8, from 0.02 to 1.04-1.14 deg;
triangle -> M1 four, from 0.01-0.05 to 0.9-1.4 deg at LAND, and the M1 four hovers at 1.44-1.50 deg when it is flown
from takeoff (X1 h4). A join that makes it even levels the ring (T -> square: 6.6-8.3 deg to 0.04-0.31 deg within the
10 s blend; 5 -> 6 and 7 -> 8, from ~1.1 to 0.00-0.05).

"No fold-in" against "fold-in" for a join has never been flown, and no switch flies it (section 3).

Two join-specific findings (section 4):
- The integral is frozen for the whole rejoin, approach and blend together (25-35 s), against 1.5 s at a detach. So a
  joined formation reaches its settled tilt about 10 s later. If joins follow one another inside each other's holds, a
  stale integral force is carried through all of them: R2768 leaned 2.4-3.2 deg in formations that otherwise hang at
  0.02-1.1 deg.
- The weld-height cap of the rejoin is not applied to QUAD4's fold-in. QUAD4 joining a pair (pair -> T, R2829) lifted
  plate 9 by 3.9 cm and the ring peaked at 9.9 deg, which is R2584's failure before the cap.

## 1. Every flown join (question 1)

Tilt = the magnitude of the ring's tilt vector (`release_bands.tilt_vector`). "Low side" = the plate the ring's lowest
edge points to (clockwise numbering, plate = 30 deg). Pre = the mean vector over the 5 s before the weld. Peak = the
largest magnitude from the weld to +15 s (or the next event). Settled = the mean vector over weld +10 .. +30 s, cut at
the next event. LAND = the mean vector over the last 5 s before LAND. QUAD4 runs land 8-10 s after the weld, at the end
of the fold-in blend, so for them "at LAND" is the mean over the last 1 s, and nothing after the blend exists. Shares =
the planner's planned tensions (log t0..) after the blend.

| Join | Runs | int_k_xy, ring | Pre (deg), low side | Peak in fold-in (deg) | Settled +10..30 s (deg), low side | At LAND (deg) | Planned shares after the fold-in (N) |
|---|---|---|---|---|---|---|---|
| T -> square, rejoin of plate 9 (E1) | R2578, R2579, R2592, R2596 | 0, 0.86 kg | 0.75 / 0.74 / 0.75 / 0.73, plate 9 | 1.27 / 1.30 / 1.45 / 1.55 | 0.04 / 0.06 / 0.05 / 0.04 | 0.05 / 0.06 / 0.08 / 0.07 | 2.50-2.51 each |
| pair -> T, rejoin of the rider, plate 3 (E2) | R2584 (no cap), R2589, R2594, R2595 | 0, 0.86 kg | 0.75 each, plate 3 | 10.61 / 3.02 / 1.45 / 2.25 | 0.74 / 0.73 / 0.72 / 0.76, plate 9 | 0.76 / 0.76 / 0.78 / 0.79 | pair 4.94-4.97, rider 0.10 |
| pair -> T, the pair leaning +3 deg toward plate 3 (E5) | R2602, R2712 | 0, 0.86 kg | 3.01 / 3.00, plate 3 | 3.12 / 5.28 | 0.45 / 0.47, plate 9 | 0.51 / 0.52 | pair 4.94, rider 0.10 |
| pair -> T, leaning -3 deg, the torque on to LAND (E5 mirror) | R2603, R2713 | 0, 0.86 kg | 2.95 / 3.02, plate 9 | 2.91 / 3.37 | 1.90 / 1.89, plate 9 | 1.87 / 2.01 | pair 4.94, rider 0.10 |
| 8 quads: 4 -> 5 (+drone 7) | R2588, R2597, R2766, R2767 | 5.26, 0.9 kg R 0.5 | 0.02-0.03 | 1.26 / 1.41 / 1.32 / 1.34 | 1.16 / 0.80 / 0.81 / 0.80 (20 s window); last 5 s 1.13-1.14, plate 7.5 | - | 1.73 / 2.83 / 2.83 / 1.73, newcomer 1.50 |
| 8 quads: 5 -> 6 (+drone 3) | same | 5.26 | 1.13-1.14, plate 7.5 | 1.48 / 1.77 / 1.74 / 1.75 | 0.03 / 0.35 / 0.34 / 0.35; last 5 s 0.01-0.05 | - | 1.77-1.79 each |
| 8 quads: 6 -> 7 (+drone 5) | same | 5.26 | 0.01-0.05 | 1.10 / 1.17 / 1.11 / 1.18 | 1.04 / 0.63 / 0.63 / 0.64; last 5 s 1.03-1.06, plate 10.5 | - | 2.00 x2, 1.22 x2, 1.61 x2, newcomer 1.06 |
| 8 quads: 7 -> 8 (+drone 1) | same | 5.26 | 1.04-1.06, plate 10.5 | 1.07 / 1.65 / 1.56 / 1.58 | 0.01 / 0.19 / 0.19 / 0.20 | 0.01 / 0.00 / 0.03 / 0.02 | 1.34-1.36 each |
| 8 quads on the lapped figure-8: 4->5, 5->6, 6->7, 7->8 | R2779 | 5.26 | 0.36 / 1.02 / 0.48 / 0.57 | 1.41 / 1.08 / 1.91 / 1.05 | 0.96 / 0.02 / 1.03 / 0.01 | 0.44 (path running) | as above |
| QUAD4: T -> square (T flown from takeoff) | R2825 (0.86 kg); R2843, R2844, R2862 (1.15 kg) | 5.26 | 7.25 / 7.80 / 8.03 / 8.27, plate 9 | none above pre (7.85 / 7.43 / 7.89 / 8.72 at the weld) | not flown (LAND at +10 s) | 0.31 / 0.20 / 0.09 / 0.27 | 2.32-2.50 (0.86 kg), 3.18-3.49 (1.15 kg) |
| QUAD4: T -> square (T after QUAD4's own soft detach) | R2826 | 5.26, 0.86 kg | 6.58, plate 9 | 5.12 | not flown | 0.04 | 2.32-2.55 |
| QUAD4: pair -> T (2 -> 3) | R2829 | 0, 0.86 kg | 0.54, plate 3 | **9.93 at +1.4 s** | not flown | 1.22 (still falling) | pair 4.94, QUAD4 0.10 |
| QUAD4: triangle -> M1 four (1.15 kg) | R2866, R2868, R2874 | 5.26 | 0.03 / 0.03 / 0.05 | 1.81 / 1.81 / 1.71 at +2.4-2.8 s | not flown | 1.41 / 1.31 / 1.38 | plate 3 5.06, 11 2.94, 7 2.95, QUAD4 (9) 2.10 |
| QUAD4: triangle -> M1 four (0.86 kg; floor join, the "nr" re-joins, full M1) | R2308, R2419, R2695, R2819, R2820, R2698, R2699, R2739, R2742, R2743, R2421 | 5.26 | 0.01-0.14 | 1.55-1.83 (R2421 2.62) | not flown | 0.22-1.15 (R2421 0.60) | plate 3 3.72-3.98, 11 and 7 2.2-2.5, QUAD4 1.16-1.65 |

Not readable: R2867, R2869-R2873 and R2875 (the 50-60 ms delayed twin and partial_step) never welded. R2830 and R2831
(T -> square at `int_k_xy` 0) never welded either: his backend waited, cause not found, so the cleanest attach-side
comparison of the gain is missing. The X3/X3b chains of 30 Oct (R2981, R2983, R2986, R2990, R2992, R2995; 8 and 9
quads) are not on the laptop yet. The 9-quad chain makes every formation uneven and should be added when it is pulled.

Does it depend on how uneven the formation is? Yes, through the formation the join leaves, not through the one it starts
from. The settled tilt of a join is the settled tilt of the formation after it:

| Formation after the join | Settled tilt reached by a join (deg) | The same formation reached by a detach or from takeoff (deg) |
|---|---|---|
| square (even) | 0.04-0.06 (E1), 0.04-0.31 at the blend's end (QUAD4) | 0.0-0.2 (every square hover) |
| 6 of 8, symmetric | 0.01-0.05 | 0.02-0.04 (same runs) |
| 8 of 8 | 0.00-0.03 | 0.01-0.04 (same runs) |
| 7 of 8, one gap | 1.03-1.06 | 1.01-1.03 (same runs) |
| 5 of 8 (0,2,4,6 + 7) | 1.13-1.14 | 1.14-1.16 (same runs) |
| M1 four (plates 3/7/9/11), 5.26 | 0.9-1.4 at the blend's end, rising (integral frozen) | 1.44-1.50 from takeoff (R2708, R2918, R2934; int_fy +0.27 N) |
| T 0/3/6, `int_k_xy` 0 | 0.72-0.79 (E2) | 0.52-0.75 by a soft detach (R2312, R2821, R2829, R2578-R2596), 0.60 by an unannounced release (R2823) |
| T 0/3/6, `int_k_xy` 5.26 | **not flown** (section 5, run 1) | 6.6-10.5 (R2965, R2972, R2249, R2826, R2825, R2843, R2844, R2862) |

## 1b. The 0.1 N tension floor (the detach agents' cause) on the join side

The detach agents' cause is the planner's tension floor. Every cable is planned at >= 0.1 N (planner_ocp.py:63
`tension_min`, geometry.py:242 `t_min`). A carrier whose level share is zero, the middle carrier of a T, is therefore
planned at ~0.1-0.13 N, and the ring's only roll stiffness, m g attach_z (0.17 N m/rad), turns that into 7-9 deg. The
coordinator asked whether any join leaves a carrier at a near-zero level share, and whether such a join shows the same
floor effect.

- **Joins that leave a near-zero share:** only pair -> T, the rejoin of the rider (E2 R2584 / R2589 / R2594 / R2595, E5
  R2602 / R2603 / R2712 / R2713) and QUAD4 joining a pair (R2829). Every other join leaves at least 1.06 N on its lightest
  carrier: square 2.3-3.5 N each, 8-quad uneven 5 and 7 at 1.50 and 1.06 N, M1 four with QUAD4 at 1.16-2.10 N. None of
  those shows a floor effect (section 1).
- **The pair -> T joins plan the rider at the floor:** 0.099 N (E2, E5) and 0.128 N (R2829), the same as the detach's T
  (R2965 0.132, R2972 0.120). **Their settled tilt is 0.72-0.82 deg, not 7-9.** The same holds for every T at
  `int_k_xy` 0, whether reached by a detach or by a join:

| T phase (window) | int_k_xy | Rider planned (N) | Ring centre offset (m, toward the rider = -y) | Settled tilt (deg) | int_fy (N) |
|---|---|---|---|---|---|
| E2 join R2584 / R2589 / R2594 / R2595 (weld +10..+29 s) | 0 | 0.099 each | -0.063 / -0.064 | 0.75 / 0.75 / 0.73 / 0.82 | 0 |
| E5 join R2712 (+3 deg lean) / R2713 (-3 deg, torque on) | 0 | 0.099 | -0.063 | 0.56 / 1.88 | 0 |
| QUAD4 join R2829 (pair -> T, the 10 s to LAND) | 0 | 0.128 | -0.065 | 0.81 | 0 |
| soft detach R2578 / R2579 / R2312; unannounced release R2823 | 0 | 0.099 | -0.064 | 0.74 / 0.72 / 0.75 / 0.66 | 0 |
| unannounced release (X1 sqT) R2965 / R2972; R2249 | 5.26 | 0.132 / 0.120 / 0.129 | +0.001 / -0.001 / +0.001 | 8.60 / 10.06 / 6.75 | -0.28 / -0.30 / -0.24 |
| soft detach R2826; from takeoff R2825 / R2843 | 5.26 | 0.130 / 0.127 / 0.091 | 0.000 | 6.89 / 6.98 / 7.90 | -0.26 / -0.25 / -0.35 |
| no resize (square model) R2966 | 5.26 | (no T in the model) | 0.000 | 2.17 | -0.30 |

(Tilt here is the planner log's `tilt_deg`, a window mean. Section 2's table uses run.csv's mean tilt vector, so the values
differ by up to 0.3 deg.)

Reading: the floor is present at both gains and in both directions. With `int_k_xy` 0 the rider's floor pull is taken by
the ring moving ~6.4 cm toward the rider (the pair's cables lean), and the ring stays within 0.8 deg of level. With 5.26
the integral learns -0.24 to -0.35 N to put the ring back on its point. The sideways give is gone, so roll (m g attach_z)
is the only compliance left, and the T tilts 6.8-10 deg. The coordinator's 0.13 N x 0.225 m / 0.17 N m/rad = 9.9 deg
(7.6 deg at 0.10 N) matches the 5.26 rows. **So the floor alone does not tilt the T in these runs. The floor together with
the xy integral does, by a join (no join at 5.26 has been flown) as by a detach.** A fix that only lifts the floor and leaves
the integral pinning the ring's xy would be the first thing to check against these rows. Section 5's runs 1-2 test both
directions directly.

## 2. The xy integral is what tilts the T (and part of every uneven formation)

| The T 0/3/6 | int_k_xy | Settled tilt (deg), low side | Ring on its xy point? | Planned xy force on the ring (int_fy, N) |
|---|---|---|---|---|
| unannounced release, resize (X1 sqT ours) R2965 / R2972 | 5.26 | 8.30 / 10.52, plate 9 | yes (y +0.005 m) | -0.31 |
| unannounced release, resize, R2249 (21 Oct) | 5.26 | 6.61, plate 9 | yes | -0.24 |
| soft detach of QUAD4, R2826 | 5.26 | 6.61, plate 9 | yes | -0.26 |
| flown from takeoff, R2825 / R2843 / R2844 / R2862 | 5.26 | 7.25 / 7.80 / 8.03 / 8.27, plate 9 | yes | -0.25 / -0.35 / -0.35 / -0.35 |
| X1 sqT no resize (square model kept) R2966 / R2973 | 5.26 | 2.19 / 2.19, plate 9 | yes | -0.35 |
| unannounced release, resize, R2823 (27 Oct) | 0 | 0.60, plate 9 | no | 0 |
| soft detach, R2312 / R2821 / R2829 / R2578-R2596 | 0 | 0.70 / 0.68 / 0.52 / 0.73-0.75, plate 9 | no: 6.5 cm toward the rider (R2578 y -0.065 m) | 0 |
| **join** (rejoin of the rider), R2584 / R2589 / R2594 / R2595 | 0 | 0.76 / 0.76 / 0.78 / 0.79, plate 9 | no (R2594 y -0.064 m) | 0 |

How it reads: in a T the rider is planned at ~0.1 N (the centre of mass sits on the pair's line), and the real ring
hangs 6.5 cm off its hover point toward the rider. With `int_k_xy` 5.26 the integral learns a 0.24-0.35 N sideways force
that pulls the ring back onto its point, and the T leans 6.6-10.5 deg doing it. The square model of the no-resize arm has
the same learned force (-0.35 N) but leans only 2.2 deg. A plausible reading is that it believes a fourth cable on plate 9
can supply the sideways pull. The offline model check can confirm this. Each source file differs only in `int_k_xy`
among the parameters that act in a steady hover (`s_chain_pair.yaml` and `chain_pair_noxy.yaml` are otherwise
identical). The runs come from different trees and days, though, so section 5 asks for the clean comparison.

Uneven 8-quad formations (5.26): the same mechanism, smaller. Over weld +10..15 s, with the integral still frozen at the
even formation's ~0, the joined 5 and 7 hang at 0.08-0.35 deg. Once the integral resumes (int_f +-0.15..0.17 N per axis)
they go to 1.03-1.14 (fig 1). Most of the uneven formations' ~1.1 deg is therefore the xy integral as well.

## 3. Is there a "no resize" for a join? (question 2)

**No switch exists.** `resize_delay_s` acts only on a detected loss (`_on_unannounced`, dissipative_node.py:2006-2017).
Every join folds the newcomer in at once:
- **Rejoin (our planner's carrier, twin only):** `_rejoin_approach_tick` (dissipative_node.py:3666) calls
  `_rejoin_fold` (3734) when the tether joint reports attached (~3688). That calls `_attach_ocp(d, rejoin=...)` (3251),
  which grows the OCP (3307) and starts the 10 s blend (3351, 3380). `_rejoin_cap` (3761, applied in `_publish_ref`
  4524-4525) holds its reference at or below the weld height through the blend.
- **QUAD4 (Tejen's newcomer):** `_magnet_attached_cb` (2242) calls `_do_attach` (3121), which calls `_attach_ocp` at
  once in OCP mode (3149-3151). Until the weld his controller flies QUAD4. At the weld our mux takes it, and nothing of
  ours would fly it without the fold-in.
- Network-mode options (`attach_central`, `attach_handout`) are not reachable with `reconfig_mode ocp`.

**The smallest twin-only switch** (not written; it needs Wesley's word, as `resize_delay_s` did): `fold_in_delay_s` on the
rejoin path only. It would be validated like `resize_delay_value` (>= 0, sim time only), refused outside the twin's
mode:=dissipative with `rejoin_departed` and `reconfig_mode ocp` (real_mode.py PLANNER_OPTIONAL), and default `''`.
In `_rejoin_approach_tick` the call to `_rejoin_fold` would wait until joint time + delay (999 = never). Until then the
rejoiner stays in mode 'weld' and keeps flying its approach hold at the weld point. The tip loop trims its body so the
rod's end stays on the plate, its cable feed-forward is zero, and its tracker still applies the measured pull
(tracker_node.py:1843-1880 is gated on authority and airborne only, not on being in the planner's model). Physically it is
an unmodelled carrier that holds its plate where it welded. Already handled: at LAND `_rejoin_out` opens the joint before
the climb in 'weld' mode (3842-3843), the drop rule does not count it (not in `slot2drone`), and the joint timeout only
fires if the joint never attaches. **One trap:** the approach tick renews `_reconfig_hold_left` every tick (3672), and that
freezes the model integral (planner_node.py:803). As written, a no-fold-in arm would fly the rest of the flight with the
integral frozen while ours unfreezes at the blend's end, which makes two variables. The switch must stop renewing the hold
once the joint is attached, or the comparison must say so. Size: ~15 lines + a test. A QUAD4 version would need a hold
reference for QUAD4 after the mux switch, which is more code. The rejoin version is enough for the twin's question.

## 4. Code differences between the two directions (question 3)

Line numbers are in the current tree (week8 5a7ccd0 + working copy). The logic was the same in the trees the runs flew.

| What | Detach (n -> n-1) | Join (n -> n+1) | Matters for the settled tilt? |
|---|---|---|---|
| Solver swap, warm start | `resize_fleet` (planner_node.py:1808), `last_X = None` (1864), `_prime_solver` x2 (dissipative_node.py:1709-1710) | the same (`resize_fleet` at 3307, `_prime_solver` x2 at 3381-3382) | no: identical |
| Model integral hand-over (decision 8 Oct) | kept: `_apply_load_force()` in `resize_fleet` (planner_node.py:1869-1872) | kept, the same call | no: identical |
| **Integral freeze** (`gz` needs `_reconfig_hold_left <= 0`, planner_node.py:803) | 1.5 s (`_reconfig_hold_s`, dissipative_node.py:1759) | **the whole rejoin**: renewed at the start (3616) and every approach tick (3672), then max(blend, 1.5) = 10 s (3397), kept while the blend runs (3793-3803). QUAD4: max(attach_traj_hold_s, 1.5) = 10 s (3406) | **delays** the settle by ~10 s; the end state is the same (fig 1). A stale force is carried if joins chain inside holds (R2768: 2.38 / 2.66 / 2.54 / 3.23 deg in the 4, 5, 6 and 7 against R2766's 0.02 / 1.14 / 0.05 / 1.04; int_f stuck at the 5-ring's (+0.17, -0.17) N from 87 to 180 s) |
| Slot geometry | survivors keep their own points (1667-1670) | rejoin: the plate point it left with (3281); QUAD4: its measured catch point, clamped to r + 0.10 (3281-3305; R2843: z 0.036 against the plate's 0.025) | no measurable effect |
| Nominal cable directions | `nominal_cable_dirs` (planner_node.py:1848) | the same, then the newcomer's replaced by `apex_direction` (dissipative_node.py:3314) | no: the same for a newcomer on the ring radius (E2's T and sqT's T get identical shares, 4.9/4.9/0.10) |
| Nominal shares | `ReferenceBuilder.__init__` `balanced_tensions(..., rod_mass, rod_lam)` (reference_builder.py:97-98) | re-solved at 3316 **without** rod_mass / rod_lam | latent only: rod_mass is 0 in every run |
| Tension floor | `tension_min` 0.1 (planner_ocp.py:63), `t_min` 0.1 (geometry.py:242) | the same, and the newcomer starts its blend at `attach_t_start_new` 0.1 N (dissipative_node.py:3361) | identical: the T's rider is planned at 0.10-0.13 N either way (section 1b) |
| Tension references after the resize | unannounced: a step to the new split; soft: the unload ramp then `post_blend` (1703-1705) | a 10 s blend from (incumbents' old shares, newcomer 0.1 N, its measured rod direction) (3361, 3379-3380), incumbents re-balanced along it (`rebalance_incumbents`, reference_builder.py:160-161) | transient only |
| Reference cap | none | `_rejoin_cap` (3761) on rejoins only: `_rejoin_blend` is set only in `_rejoin_fold` (3757). **Not on QUAD4's fold-in** | transient: R2829 (QUAD4 pair -> T) reference +3.9 cm in 1.4 s (2.065 -> 2.104 m), ring 9.93 deg, as R2584 (+4.6 cm, 10.61 deg) before the cap; with the cap R2589 0.86 deg at the same instant |
| Drop rule | the leaver leaves the count | a rejoiner counts as absent until its blend ends (3793) | no |

So nothing in the resize code differs between the directions in a way that changes the settled tilt. The differences
change the transient: the 10 s blend, the cap (which QUAD4 lacks), and the integral frozen for ~10 s longer on a join.

## 5. Runs for the lab chat to queue (question 4)

The gain is the open variable, and no batch varies it alone. Three configs, written and checked
(`python3 tools/run_experiment.py --check` OK. The only warnings are load_ixx / load_izz world vs controller, inherited
from the base configs). All are exploratory rows with their falsifiers in the description. One tree for each pair: fly the
baselines again in the same batch.

| # | Config | Against (re-fly once each on the same tree) | Variable | Prediction (falsifier) | Cost |
|---|---|---|---|---|---|
| 1 | `configs/experiments/rj3_pairrelease_rejoin_L100_tip_kxy.yaml` x2 (params `configs/twin/chain_pair_tip_kxy.yaml`) | `rj3_pairrelease_rejoin_L100_tip` (R2589 / R2594 / R2595: 0.72-0.79 deg) | `int_k_xy` 0 -> 5.26 on a T reached by a JOIN | the joined T settles 6-10 deg like every T at 5.26 (falsified if it settles under 3 deg in both) | 3 quads, ~10 min wall each, 4 slots |
| 2 | `configs/experiments/x1_sqTnoxy_ours.yaml` x2 (params `configs/twin/x1_sqT_noxy.yaml`) | `x1_sqT_ours` (R2965 / R2972: 8.30 / 10.52) | `int_k_xy` 5.26 -> 0 on X1's resize arm | settles near R2823's 0.60 (falsified if over 3 deg) | 4 quads, ~9 min wall each |
| 3 | `configs/experiments/x1_sqTnoxy_nors.yaml` x2 | `x1_sqT_nors` (R2966 / R2973: 2.19) | the same, no-resize arm | open: whether "no resize beats the resize" survives without the xy integral (falsified if it settles > 1 deg under #2 in both) | 4 quads |

Run 1 is the attach side's missing cell (a join into a T with the xy integral on). Runs 2-3 are the detach side's mirror
and are written here only because they decide the same question. Total 6 runs + 3 baseline re-flies, ~40 min on 4 slots.
Optional: the 8-quad chain at `int_k_xy` 0 (`s8_chain4back_R50_L100_it5` with `params_file`
`configs/rig/chain_pair_noxy.yaml`, which exists) would show how much of the uneven 5/7's ~1.1 deg is the integral (~25 min
wall each). Not written. Fly the no-fold-in arm only after the gain runs, and only if Wesley approves the switch in
section 3.

## Caveats
- The QUAD4 joins land at the end of the 10 s blend with the integral frozen, so their "after" is a blend-end value,
  not a settled one. The M1-four value (1.44-1.50) is from takeoff hovers, not post-join.
- Twin only: rigid hinged rods, no added delay. The rider's "height is the pitch" with a rigid rod (critic, 23 Oct). On
  the rig the cables can slacken, and the rig flies the chain files with `int_k_xy` 0.
- The gain comparison spans trees (23 Oct - 30 Oct), worlds (L100 square, attach-layout worlds) and rings (0.86 / 1.15
  kg). Within each group the pattern holds, but no single batch varies the gain alone (section 5).
- The 8-quad shares after a join are 1-4 % above the detach's for the same formation. The z integral grows over the
  flight (int_fz -0.37 -> -0.55 N), so this is not a share difference. Proportions match to 3 decimals.

## Decisions for Wesley
1. Queue section 5's runs 1-3 (6 runs + 3 re-flies, lab PC, ~40 min)? Recommended: they decide whether the sqT result is
   "the resize" or "the xy integral in a T", for both directions.
2. If run 2 confirms: which `int_k_xy` should the twin's claim cells fly for uneven formations? The rig's chain files
   already fly 0. Changing the default is a launch default, so it needs your word. A fix to the tension floor alone
   (section 1b) should be checked against the `int_k_xy` 0 rows first: at 0 the floor is already harmless.
3. Should QUAD4's fold-in get the rejoin's weld-height cap (R2829: 9.9 deg on pair -> T in the twin)? This is a change
   in the flight loop, so it needs the critic. On the rig the slack cables may not show it.
4. `fold_in_delay_s` (section 3): build it as a twin-only baseline arm, or not needed once the gain runs are in?
