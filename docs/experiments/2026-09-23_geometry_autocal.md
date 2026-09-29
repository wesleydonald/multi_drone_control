# Card — per-drone rod length from measurement, and a tautness gate that does not ramp on a typed number (2026-09-23, v2)

v1 of this card (disc-centre and magnet-azimuth inference from a 3-drone circle fit) was
withdrawn after the critic's challenge, kept at the end. v2 is the identifiable part.

## 1. Question
Wesley: "I don't want to need centimetre-accurate cable lengths in real life; the code
should adjust for that." Does measuring each rod once it is taut, and using that length
instead of the typed `cable_len`, remove the 20 cm hover sag a 4 cm rod error causes?

## 2. Hypothesis
Two mechanisms make the planner sensitive to `cable_len` (SIL, one variable each):

| told / real | hover z (target 0.60) | mechanism |
|---|---|---|
| 0.50 / 0.50, R0275 | 0.597 m | — |
| 0.50 / 0.46, R0286 | 0.397 m | tautness gate ramps the tension feedforward from 0 at 85 % to 1 at 100 % of the TYPED length; 92 % passes ~47 % of it |
| 0.46 / 0.50, R0287 | VOID (solver compiled mid-run, drones never flew) | expected: references 4 cm inside the rods' reach; measured in the frozen-tree matrix below |

Hypothesis: with rods rigid, the drone-to-rim distance at the moment the OCP takes over
(creep handover, or a taut air start) IS the rod length. Setting each drone's `l_i` to
that measurement, and computing the gate against it, makes both cases hover within 3 cm
of the correct-geometry hover. When the typed length is right it changes nothing.

## 3. The one variable
Planner parameter `measure_rod_len` (default false = today). At `_enter_planner_phase`,
after slot assignment and with the creep arc already finished (rods taut by
`TAUT_SWITCH_GATE`, or a taut air start): `l_i = |drone_i − rim_i|` per drone; guard: each
within ±15 % of the typed `cable_len` and all three within 3 cm of each other, else keep
the typed value and WARN. Apply via `PlannerSolver.set_geometry(cable_lengths=l)`, and the
tautness gate uses `l_i` instead of `cable_len`. No change to rho, azimuths, the creep
controller (it has finished), or the dissipative network. Logged as one line and in the
`params/` read-back. Slack at rest is not a risk: the measurement is taken after the creep
has swept the rods taut, not from the resting placement.

## 4. Baseline and arms
SIL (free): `carry_hover_n3_told50_real46` and `carry_hover_n3_told46_real50` with the
feature off (R0286/R0287) vs on; plus `carry_hover_n3` (told = real) with it on, which must
equal R0275 within 1 cm (no-op check). Gazebo: `three_rigid_ground_l046.sdf` (rods 0.46,
generator `--cable-len 0.46 --ground-start --detachable`), planner told 0.50, feature off
vs on, 2 repeats each; the correct-geometry reference on that world is the SIL number,
not a third Gazebo arm.

## 5. Pass / fail numbers
| arm | supports | falsifies |
|---|---|---|
| SIL, feature on, both mis-seeds | hover within 3 cm of R0275's 0.597 m, tilt settled < 3° | either mis-seed still > 8 cm low or oscillating |
| SIL, feature on, told = real | within 1 cm of R0275 | any difference > 1 cm (not a no-op) |
| Gazebo, feature off, rods 0.46 told 0.50 | hover < 0.45 m or abort, 2 of 2 | hover ≥ 0.52 m (Gazebo does not show the sensitivity; then it is a SIL-only finding) |
| Gazebo, feature on, same | hover within 3 cm of the same-world correct-geometry expectation 0.537 m (R0284 at 1.0 kg was 0.537; R0276 0.566 at 0.6 kg) and no abort, 2 of 2; logged `l_i` = 0.46 ± 0.01 | hover < 0.50 m or abort in either |

## 6. Repeats
2 per Gazebo arm; SIL runs are free and single.

## 7. Cost
4 Gazebo runs of 6 (0 used today). SIL first; if SIL falsifies, no Gazebo runs.

## 8. New code?
Yes: ~30 lines in `planner_node.py` (measurement, guards, per-drone gate), one launch arg
on the OCP launches (sim and real), a unit test for the guard. Removal plan: delete the
block and the arg. Not in scope: rim azimuths, payload origin offset (pre-flight's disc fit
reports them; the placement rule and the mocap pivot handle them).

## Reviewer, SIL half, first pass (2026-09-23): not supported as worded

Accepted and corrected:
1. R0287 was void: its acados solver compiled during the run, the drones never flew,
   and the "oscillating 0.30–0.44 m" was a payload dangling under parked drones. Dropped.
2. Every quoted height averaged the descent into the hover: `metrics.steady_window` ran
   from TAKEOFF + 5 s to the END of the record with no LAND cutoff. Fixed the same day
   (window ends at the LAND event; `tools/test/test_metrics.py`); hover-only, the
   correct-geometry SIL hover is 0.634 m, not 0.597, and the fleet sits 3.4 cm HIGH.
3. The +20 % mass case cannot separate kT from the OCP model mass: one `load_mass` arg
   sets both, kT was 34.57 in every run, the trackers held their references to 0.8 mm
   and the planner's own reference dropped. Restated as a joint error, 9.4 cm.
4. No repeats and a working tree that changed between arms. Re-run as a frozen-tree
   matrix, ten configs × 2 repeats, solver prebuilt, results below.
5. SIL `params/` read-back is empty on the bench (pre-existing gap); the flag was
   verified from the launch line and `manifest.json`.

## Critic

**1. Already built.** `tools/preflight.py:152-175` is this exact estimator, with the same
"valid with the rods flat on the floor" caveat. "Measured drone-to-rim distance as the
controller's length" is a §6 negative (R0261-R0263: the 0.53 m reading was the tip
stand-off). Card step 4 is that quantity again.
**2. Slack and tilt.** A drone placed 5 cm inside its reach reads short; autocal then
shortens l_i and manufactures R0287 with the ±15 % guard wide open. Step 3 takes a world
bearing and never de-rotates by psi0 (`geometry.py:105-125`).
**3. n=3 is not identifiable.** 6 numbers, 8 unknowns (centre 2 + 3 azimuths + 3 lengths).
The circle through 3 points is exact, residual identically 0; unequal rods give a
confidently wrong centre, indistinguishable from an origin offset.
**4. Ordering.** Calibration must precede the creep controller latching its arc anchors;
at the "first solve" the drones sit at 45°, not resting. Changing rho after
`_assign_slots` scrambles `slot_az`. `set_geometry` itself is safe.
**5. Sim arm.** Generated worlds are radial and even, so the 20-50° non-radial error is not
reproduced; with `cable_len` 0.50 on 0.46 m rods the arc reference commands past reach,
so the baseline likely fails as the liftoff deadlock and passes "hover z < 0.45" spuriously.
**6. Bands.** Correct geometry hovers 0.537-0.566 m today, so autocal can miss its own
0.55 bar for an unrelated reason. No good-geometry autocal arm tests that it is a no-op.
**7. Cheaper first.** Per-drone l_i = measured distance at handover, or rigid-rods-are-taut
/ a wider `CABLE_TAUT_LO_FRAC`, addresses the row-1 mechanism in ~5 lines; or run preflight
and type the fitted numbers. Wesley asked for tolerance to centimetre errors, not for
inferring the disc centre and azimuths from mocap.
Verdict: not ready — the n=3 fit is underdetermined and self-confirming, the guards do not
catch the slack-rod case, and the pass band collides with the unresolved hover offset.

Response (2026-09-23): accepted in full. v1 (centre + azimuth inference) withdrawn; v2
below keeps only the identifiable part. The tip stand-off negative (R0261-R0263) concerned
the newcomer's measured rod in the network; here the measurement is drone body to the
planner's own rim point, which is the quantity the OCP constrains.

## Outcome (2026-09-23, SIL half)

Frozen tree f8e2ef4 + 26 dirty (identical `diff_sha256` on R0308–R0327; R0307 ran four
minutes earlier on 24 dirty and is not part of the matrix), solver prebuilt, 0 compiles,
hover-only window (8–16 s, LAND at 16.0 s in every run), `fleet_aborts: []` everywhere.
Target 0.60 m. Two repeats per config; worst repeat disagreement 0.4 mm.

| config | feature | hover z (m) | measured rods | runs |
|---|---|---|---|---|
| correct geometry | off | 0.634 / 0.634 | — | R0308 / R0309 |
| correct geometry | on | 0.634 / 0.634 | [0.500]×3 | R0314 / R0315 |
| rod 0.46, typed 0.50 | off | 0.416 / 0.416 | — | R0310 / R0311 |
| rod 0.46, typed 0.50 | on | 0.631 / 0.631 | [0.460]×3 | R0316 / R0317 |
| rod 0.50, typed 0.46 | off | 0.569 / 0.569 | — | R0312 / R0313 |
| rod 0.50, typed 0.46 | on | 0.634 / 0.634 | [0.500]×3 | R0318 / R0319 |
| rim 0.27, typed 0.25 | on | 0.642 / 0.642 | [0.514]×3 | R0320 / R0321 |
| rim 0.23, typed 0.25 | on | 0.626 / 0.626 | [0.486]×3 | R0322 / R0323 |
| load 0.72 kg, typed 0.60 | on | 0.540 / 0.540 | [0.500]×3 | R0324 / R0325 |
| rod 0.46 + rim 0.23 + 0.72 kg | on | 0.549 / 0.549 | [0.474]×3 | R0326 / R0327 |

§5 SIL rows, restated against the hover-only baseline 0.634 (R0308/R0309; the card's
0.597 was R0275 with the landing averaged in): feature on, both mis-seeds within 3 cm and
tilt < 3° — met (0.631, 0.634, tilt ≤ 1.6°); no-op within 1 cm — met (0.05 mm). Neither
falsifier (> 8 cm low or oscillating; not a no-op) occurred. Measured rods match the
45° geometry to 0.4 mm in every case. Settled tilt 0.7–1.6°, tension-share spread < 7e-4
in every run except feature-off short-rod (2e-3; those two also exceed the peak
cable-acceleration bar, 8.6/8.3 vs 8.0). Mass rows: the plant was 20 % heavier than the
typed `load_mass`, which sets both the OCP model mass and the derived kT, so the 9.4 cm
sag is not attributable to either; not addressed by this feature (open problem 9).

Reviewer (second pass): weak as first worded; four corrections, none touching a height:
(1) tension-share bound was overstated for R0310/R0311; (2) R0307 is not on the frozen
tree, R0309 is the repeat of R0308; (3) the mass mechanism sentence claimed the launch
arg moved when only the bench plant did; (4) 0.4155/0.4159 rounds to 0.416. `metrics.json`
`steady_window_s` reported the record end instead of the LAND cut (cosmetic, fixed same
day, all 21 regenerated). Claim as corrected above: supported.

Not done: the Gazebo half (§5 arm 3, 4 runs) and the launch-default flip. Both are
Wesley's call; see `docs/decisions.md`.
