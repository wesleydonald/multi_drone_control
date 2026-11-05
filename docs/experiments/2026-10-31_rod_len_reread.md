# Card: read each rod's length under the pretension's held pull, before the lift (31 Oct 2026)

Owner Wesley (31 Oct, "build it behind a switch"). Real-flight bars R1-R3 FROZEN by Wesley 31 Oct ~17:15, before any real flight of
it. New code in the flight loop: two critic passes, section 7.

## 1. Question
The planner reads each rod's length once, at hand-over, with its pull still near zero. Real cables then read longer. Does reading them
under the pretension's held pull, before the lift, remove the ring's steady height error and the tilt that comes with unequal lengths?

## 2. Evidence (real flights, replayed offline, 31 Oct; the design changed because of it)
The first version re-read at the top of the lift. The critic (section 7) asked whether the hand-over reading is simply taken too
early, and the logs say yes. On 12 flights on 1 m cables and 21 on 0.55 m (`results/analysis/why_integral_2026-10-31/rod_hold.py`), flown length minus each reading:

| Reading | 1 m: mean, SD, range (mm) | 1 m: mean over a flight's rods | 0.55 m: mean, SD (mm) |
|---|---|---|---|
| At hand-over (today) | +5.7, 5.8, -7.1 to +15.8 | +5.7, so about 25 mm low (4.4 x) | +2.6, 2.1 |
| Last 0.5 s of the pretension's hold | -0.8, 2.4, -10.3 to +3.2 | -0.8, so about 4 mm high | -1.8, 1.8 |

sq3 (Q2, Q3): hand-over 0.986, 0.986; hold 0.999, 1.000; flown 1.000, 1.002; rods read 0.986 at +0 s and 0.9975, 0.9998 at +3 s as the pull
ramps. A re-read at the top found its 2.5 s still window in only 5 of the 12 flights, and could move the ring during Tejen's attach. The
hold is before the lift, on every floor start, on the existing timeline. Q1 is the largest residual (-10 to +2 mm): not understood.

## 3. Hypothesis
A short `l` by `d` is a steady command toward the ring along that rod, and the ring sags about 4 x `d` until the plan's own rise cancels
it. The height integral cannot help: on the measured pull it changes the planned tension, not the plan's motion (R3240 38 mm low with it at
its -2.14 N bound, R3242 40.8 mm low with it off; the planner told 10 mm short).

## 4. The change (behind a switch, default off)
`rod_len_reread` (planner option, `common.yaml` `''` = off, rig and twin). `mpc_planner/rod_reread.py` (pure): the median of `_rim_dist` (the
hand-over's own reading) over the last 0.5 s of the pretension's hold, then `cable_len_i` and the solver's geometry ramp to it over 0.5 s as
the height ramp starts. The hold's pull is not always full: 7 of 15 flights on 29 Oct broke away and held at 63-97 % of the plan, and by the
hold's end the ring may be up to 45 mm off the floor. Once per flight. A change over +20 / -8 mm (seen +16, -6), or one rod's samples spreading
over 15 mm, keeps the lengths and logs it. A resize or a descent cancels it where it is. A start with no hold (taut, airborne) logs a
cancel; a start weld's resize cancels it, so that flight keeps the hand-over's lengths. The timeline is unchanged. Off, nothing is touched.
Later resizes carry the lengths per drone, as the hand-over's; a welded newcomer stays typed.

## 5. Runs and bars
Simulated rods read exactly, so telling the planner a wrong length tests only the wiring and the limits, not the claim.

**Wiring, flown 31 Oct on the laptop (mean of the hold; the median and the spread check came after, not re-flown; registry R3402-R3404, R3410, R3411):**

| Config | Change | Bar | Flown | |
|---|---|---|---|---|
| `b3p115_len990_rr` | R3240 + the switch (told 0.990) | reads 1.000 within 2 mm; under 5 mm off in height | reads 1.000; -0.2 mm (R3240 -38.0); tilt 0.07 deg | pass |
| `b3p115_len1010_rr` | told 1.010: outside the -8 mm limit | refused and logged | refused; +39.0 mm high | pass |
| `b3p115_rr` | right lengths + the switch | within 2 mm, 0.2 deg of R2838 | -0.7 mm, 0.06 deg (-0.4, 0.11) | pass |
| `b4p115_fig8_len990_rr` | a figure-eight told 0.990 | clock-free error under 20 mm | 13.3 mm (R2835 12.9) | pass |
| `x1_h4_rr` | M1 four, one release | within 1 deg rise, 5 mm dip of `x1_h4_ours` | rise 6.96, dip 32.9 (ours 5.73, 7.16 and 29.1, 33.7) | pass |

**The claim is on real flights. FROZEN 31 Oct (Wesley): Wed's logs decide Thursday.**
- **R1, the reading (no switch needed, offline on Wed 4 Nov's h3t, h3t180 and h4m1, flown switch off):** on each flight the hold's reading within 3 mm of
  what the rods read at the top, as the mean over its rods, and at most one rod over 5 mm. The 12 flights of section 2 would pass the
  mean on 11 or more; Q1's -10 mm is the one rod. If R1 fails on Wed the switch does not fly on Thu.
- **R2, the height (first flight with the switch, a triangle hover):** ring height error at the top within 10 mm (16-37 mm low on 29 Oct's triangles).
- **R3, the tilt:** ring's mean tilt at the top no worse than the same day's switch-off flight in that layout by more than 1 deg.
- **The call (Wesley 31 Oct):** R1 holds on Wed: Thursday's first triangle hover flies the switch. R2 and R3 hold: all of Thursday's baselines fly it, one setting per table. Otherwise all off.
- **Falsified by:** R1 holding with R2 missed (the sag explanation is wrong), or a refusal on a sound rig. The bump at the ramp is reported, not barred: the hold itself
  moves the ring 1.3 to 4.5 deg with the switch off (k1_4, f4). The sheet's stop rules stay in force.

## 6. What it does not do
It leaves the room-fixed 0.2 N alone (about 1 deg; the turned hover of 4 Nov). The reading is an effective length of that plate and
formation (Q1 reads 0.978 on plate 3, 0.991 on plate 0), right at node 0 for that formation. A survivor's change after a resize stays
unread. Q1's residual stays. Low pull may read short: a T's rider at 0.1 N is unread at full pull only if the T is flown from the floor.

## 7. Critic

Challenge of 31 Oct ~15:50 of the first version, the re-read at the top of the lift (the critic, read-only). Sections 1-6 were rewritten after it.

**1. Tried?** Not in flight. Relatives: R0295-R0327 (hand-over reading), R0261-R0263 (network length, no benefit). R3240/R3242 show sensitivity, not this fix.

**2. Simplest explanation.** The floor reading is bad, not the cable: six 29 Oct triangles repeat flown lengths (Q1 0.977-0.979, Q2 0.995-1.001, Q3 1.000-1.003) while latched scatters 7-11 mm. Reading in the pretension hold (`planner_node.py:101`) may suffice; logs can test that.

**3. Falsifier.** Within one flight, ring rise after the ramp differs from 4.4x the mean vertical change. No bar tests that.

**4. Breaks.** Off, nothing. On, it can move the ring during Tejen's attach (A).

**5. Asked?** Yes (`decisions.md:750`), but his wrong-length Gazebo run only tests wiring.

**A. Code.** `_rim_dist` reads an effective length: Q1 is 0.978 on plate 3, 0.991 on plate 0 (ch1b_2). Plate slot, body origin and ring bending (~9 mm, its estimate) go in: right for node 0, that formation only. Lean: about 1 mm per cm of pivot error (its estimate). No stale copies (`planner_solver.py:272-274`, `dissipative_node.py:1690-1698, 3302, 4155`); trackers hold none. A newcomer stays typed (`:3303-3304`). "Taut" is circular: 0.985x the stale `l` passes (`planner_node.py:224-225, 2093-2095`), so a rod 15 mm under it gets locked in, likeliest on a square. No ring-rate or drone-pose-age check; the ramp ignores `ready` (`rod_reread.py:50-57`). It needs 2.5 s unbroken, and the similar integrator gate was open 28-100 %. A missed hold stays `wait`, unlogged. Attach: no QUAD4 term, no hold (`test_partner_contact.py:204-211`). A late calm can fire during his descent, handing the proof its 3-5 mm of plate motion (`comotion_proof.py:49-50, 274`). Refuse after `_his_took_off`. Failed solve: the ramp runs on; small.

**B. Physics.** No stretch visible: Q2 0.998 at 4.39 N (k1_8) and 5.26 N (sq2_5). Low pull may read short: Q3 0.552 at 0.46 N (t3p_2), 0.559 at 3.69 N (h3), different plates. So one reading, no following. Takeoff is unchanged; LAND's arcs inherit it. The 4.15x fit leaves 0-8 mm low (its estimate): frame or rest height, unfixed.

**C. Bars.** No. Simulated rods read exactly; 0.990 on every rod gives no tilt, so B's tilt cannot fail. No unequal or told-long arm. `x1_h4_rr` has right lengths, so a lost re-read cannot show. A leaves 5-19 mm undecided. D is one flight against scatter of 0.9-4.9 deg, 16-37 mm: score within a flight.

**D. Risk.** Worst: a wrong reading accepted. One rod +30 mm: about 8 deg, 60 mm sideways; all rods about 120 mm high (linear). Only tilt over 30 deg or LAND stops it. Limit +20, -8 mm (seen +16, -6). 1.5 s is fine; afterwards is not: ring time constant about 2 s, and the unfrozen sideways force pushes back up to 30 mm (its estimates), so a 5 s hold is too short. First flights: triangle hover, after replaying `RodReread` on logged flights.

CRITIC: rewrite the card | the logs blame the floor reading, and no bar tests a real one.

Second challenge, 31 Oct ~16:05, of the version in sections 1-6 (the critic; eight flights' rows read by hand, `rod_hold.py` not run, section 2's table unchecked):
1. Yes, breakaway jump included: 6 ticks, the replay's 5 plus one earlier (`planner_node.py:1514-1518`; sq3 rows 391.057-391.453). But "full pull" is false: 7 of 15 29 Oct flights held at 63-97 % (sq3 63 %, ring +3 mm), a path the twin never takes (`:1597-1599`).
2. No gate misbehaves: floor starts override the rod gate (`:2165`), detection uses max(typed, l) (`dissipative_node.py:1982`), LAND arcs start closer (`:1216`). The ring moves more than the card says: at hold end h4 +45 mm, k1_4 +44 mm, 0.1 m/s, 4 deg. The mean has no spread or pose-age check (`rod_reread.py:56`): one 60 mm glitch moves a rod 10 mm, in limits. Add a median and spread refusal.
3. No defect. A start weld's resize cancels it: hand-over lengths all flight (`planner_node.py:1852`). `drop_takes_back` reads no length. Rejoin carries the reading, the newcomer stays typed (`dissipative_node.py:1698,3304`). `measure_rod_len` off: Q1's -22 mm refuses all rods.
4. No. R1: Q1 reads about 7 mm over flown on k1_4 and sq2_5, 2 of 6 triangles, so a working switch passes 3 of 4 about 60 %; score it switch-off on Wed's h3t. R3: the predictor spans 0-11 mm against a 5 mm bar; pool with the day's switch-off h3. The 1 deg bump is exceeded switch-off in the hold (k1_4 1.3 to 4.5 deg, f4 0.2 to 4.1). No bar tests tilt.
5. A pass has no home: Thursday's ten baselines are the table (`handin_plan.md:115`), one tree per table, code freeze Fri 6 Nov. Wesley must choose the order first. `decisions.md:750` says top of the lift, not the hold.

CRITIC: rewrite the card | no code defect, but R1 and the bump bar can fail on a working switch.
