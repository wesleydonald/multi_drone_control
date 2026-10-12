# ancillary_tracker (4 Oct 2026, overnight): make the trackers execute the planner's thrust feedforward

Wesley, 3 Oct (overnight loop): remove the ring's height and xy errors from internal model errors
with a principled control-law change.

## 1. Question

Does the twin's +4.8 cm hold (and SIL's +6.3 cm with a matched plant) come from each tracker
ignoring the planner's thrust feedforward? If so, does weighting that feedforward in the tracker's
cost (the ancillary controller of tube MPC: nominal input plus feedback on the state error) remove
it without an integral?

## 2. Hypothesis

**Measured (no new flight).**

| | R0868 hold tick (rod model, drones on their planned attitude, tilt within 0.3-0.5 deg) | R0841 |
|---|---|---|
| thrust | plan's node 0 asks 15.66 m/s^2 (the descent: node 20 holds 16.2); trackers deliver 16.26 (u2 0.4619 x 35.2), the hold value | |
| attitude | | trackers fly 4.4 deg more tilt than the plan's (19.0 vs 14.6 deg) |

**The cost.** `tracker/acados.py` weights the throttle reference (the planner's thrust feedforward,
u_state index 11) at 2e-4, against position 80/80/40, velocity 2 and attitude 0.5 (W_e 0.5/0.5/5).
The tracker therefore picks its own thrust from its model and the position references. Those barely
move in the first 0.2 s of a planned descent: the plan's node 1-2 drone heights are within 2 mm of
node 0 (hold_replay R0868).

So the descent the plan starts by cutting thrust is never executed. The plan re-pins at the measured
state and asks again, and the loop sits where the trackers' position stiffness balances it.

**This fits the record:**
- R0475/R0874: FF 0.9 lowers the hold by about 10 cm; the trackers do execute the cable term in their
  model.
- The offset-free observer converges to +0.56 N (R0893): a virtual force that cuts the planned
  tension until the trackers' own model makes them descend.
- The rod-rate jump reversing sign (M-pin-replay) does not matter: the thrust cut is the same sign
  in both.

**The fix: tube MPC.** The ancillary law follows the nominal trajectory's state and input
(Mayne, Seron & Raković 2005; Rawlings, Mayne & Diehl 2017 ch. 3). The tracker weights the thrust
feedforward comparably to position, so a planned thrust cut is flown and the position error feeds
back around it.

**Expected.**

| arm | z_ki 0 | expected |
|---|---|---|
| SIL hover (matched plant) | +6.3 cm | within 2 cm at w 1, closer at w 10 |
| twin hold | +4.8 cm | within 2 cm |

No integral, so no change against an external force: push and wind need A.

**Risk.** Weaker position feedback, so more sway (lift transient, push), or worse tracking where the
plan's thrust is wrong (a map error: the map arm).

## 3. The one variable

Tracker parameter `w_thr_ref` (default 2e-4 = today's compiled weight, passed only when set). It is
the weight on u_state throttle (index 11) at every stage and the terminal stage, set at run time with
`cost_set(k, 'W', ...)`, so there is no rebuild. Values: 1.0 and 10.0 (SIL), the better one in Gazebo.

## 4. Baseline

R0473 (SIL hover 0.663), R0841 (twin hold +4.8), R0856 (push 18.0), R0870 (map), R0832 (orbit).

## 5. Pass / fail numbers

| arm | supports | falsifies |
|---|---|---|
| SIL hover | within 2 cm of 0.600 | still > 4 cm (the feedforward is not the cause) |
| twin hold | within 2 cm, tilt mean within 0.5 deg of R0841 | > 3.5 cm |
| any | lift transient (ring vz max, tilt peak) within 30 % of R0841; 0 solve failures; landed | an abort, oscillation > 2 cm p-p |

## 6. Repeats

Exploratory, one run per arm.

## 7. Cost

2 SIL + 2-4 Gazebo runs.

## 8. New code?

Yes, small: the tracker parameter and a run-time `cost_set` of W (and W_e) after the solver loads;
launch plumbing (optional knob). If it loses, it stays behind its default until Wesley decides.

## Critic

**Critic, 4 Oct 2026: NOT READY.** Rewrite the card: the fail bar cannot separate a wrong
mechanism from a weight too weak to fly the cut, and the SIL gap has never been measured.

Must-fix:
1. Offline: the plan's node-0 vertical specific thrust against the flown u*kT*R33, with free and
   pinned rates. Drop the card if the gap is below ~0.2 m/s^2.
2. A tracker re-solve sweep of w (2e-4 to 1e3) in acceleration units; how much of the cut comes
   through the x0 box.
3. A mechanism falsifier: the gap closes to < 20 % with the height still > 4 cm. Fill the 2-4 / 2-3.5
   cm bands.
4. Arms: FF 0.9, n=2, circle; a map bar; z_ki 0.4 and the observer as comparators.
5. Active only after the lift. Refused with kt_trim, dissipative, network and attach.
6. Bars: drone and tilt p-p, a slow planner tick, LAND.

Also: the tube-MPC citation does not fit (node 0 is re-pinned, so the ancillary feedback on x - x_bar
is zero), and "cost retuning" was set aside by Wesley (decisions.md, 3 Oct).

**Must-fix 1 done (registry M-ancillary-gap).** Hold ticks, vertical specific thrust (m/s^2):

| run | ring | flown (u2 kT cos tilt) | plan node 0, free rates | plan node 0, pinned | plan steady (node 20) |
|---|---|---|---|---|---|
| R0841 | +4.8 cm | 14.87 | 15.50 (gap +0.63) | 14.56 (-0.31) | 14.95 |
| R0868 | +5.0 cm | 14.88 | 14.29 (gap -0.59) | 14.69 (-0.19) | 14.92 |

The gap flips sign at the same height error. In both runs the drones fly the plan's steady vertical
thrust within 0.5 %.

**Verdict: falsified offline, not flown.** `w_thr_ref` stays at its default (0 = the compiled 2e-4).
