# Twin timing against the rig's QUAD4 swing, and Tejen's timing fix (29 Oct evening)

Card `docs/experiments/2026-10-29_rig_timing_twin_attach.md`: the triangle attach with sq5_2's settings, lab PC.

**Tejen's timing fix (partial_step) in the twin, QUAD4's climb:**

| Send path | Undelayed twin | +50 ms | +60 ms |
|---|---|---|---|
| His current send (endpoint) | normal (R2866 / R2868) | normal (R2867 / R2869) | normal (R2870 / R2871) |
| The fix (partial_step) | normal, welded (R2874) | **QUAD4 crashed** (R2875) | **QUAD4 crashed** (R2872 / R2873) |

**Verdict**
- **The fix on the 60 ms twin (E2c) is void for its swing bar.** QUAD4 never reached the plate: it bounced vertically
  as it lifted its cable, then flipped onto the floor about 7 s after takeoff, in 2 of 2 runs. [reviewer: not run]
- **The fix's send path works as Codex specified:** right profile, steps of one interval instead of 100 ms. [reviewer:
  not run]
- **The same crash happened at 50 ms (1 run) but not without the delay (1 run):** with ~50 ms of loop delay, the fix
  destabilises QUAD4's climb in this twin. [exploratory]
- **The crash did not land the fleet:** QUAD4 lay armed upside down for ~50 s.

**The swing:** the decided 50 ms twin misses the rig's (0.7 deg against 2.1). 60 ms partly reaches it, but its carriers
sink after ~50 s. The rig's ring itself moved 4.6 cm during sq5_2; the twin's 0.1-1.1 cm.

| Arm | Runs | QUAD4 swing (deg) | Magnet to plate (cm) |
|---|---|---|---|
| Rig sq5_2 | RIG-1029-sq5_2 | 2.13 | 6.15 |
| Twin, +50 ms | R2867 / R2869 | 0.69 / 0.68 | 1.86 / 2.07 |
| Twin, +60 ms | R2870 / R2871 | 1.60 / 1.80 | 3.75 / 3.67 |

**Decision:** before the rig rehearsal of the fix:
- **(a) Recommended:** first a twin run with the throttle-only delay at the rig's measured value. This shows which
  channel crashes the climb.
- **(b)** Ask Tejen to model the delay in his MPC before the fix flies.
- **(c)** Fly the rehearsal as decided, ready to cut at liftoff.

## Channel split (exploratory, 17:22-17:46; Wesley chose (a), the fix stays off the rig)
The fix flown with 60 ms on one channel group only, one run each:

| Delay | Run | Climb | Approach |
|---|---|---|---|
| Throttle only | R2876 | bounced, flipped at 29.8 s | never reached |
| Rates only | R2877 | clean (6.5 deg) | the swing grew at ~0.7 Hz, flipped at 45.8 s |

**Verdict:** the throttle delay alone flips QUAD4 in its climb, and the rate delay alone does not, as predicted.
[exploratory] But the rate delay alone flips it in the approach instead. With the fix, a 60 ms delay on either channel
destabilises QUAD4 in this twin; his current send flew both cleanly. The send path read as before (first step 0.016,
per-tick at most 0.045 / 0.0185).

## W4 debounce (exploratory, 18:51-19:10; R2878-R2881)
Tejen's 8 cm descent restart was held for 0.3 s (`descent_debounce:=0.3`), one change against each earlier arm, all on
his current send. Twin contact is the twin's 1 cm gate, not the rig's slot.

| Arm | Runs | Descents | Reached the bottom | Restarts before the bottom | S (deg) / E (cm) | Contact |
|---|---|---|---|---|---|---|
| 60 ms, no debounce (E1c) | R2870 / R2871 | 3 / 4 | first only | 2 / 3, all one tick at 7.9-8.6 cm | 1.60 / 1.80, 3.75 / 3.67 | none |
| 60 ms + debounce | R2878 / R2879 | 4 / 1 | 2nd / 1st | 3 / 0 | 2.45 / 4.75, 5.77 / 7.35 | none |
| 50 ms + debounce | R2880 | 1 | yes, held 26 s | 0 | 1.11, 2.31 | none (as E1) |
| No delay + debounce | R2881 | 1 | yes | 0 | 0.49, 0.85 | welded 46.2 s (as E0) |

**Verdict** [exploratory]
- **The debounce did what it says:** no single-tick 8 cm restart remained.
- **The 60 ms descents still restarted** (3 of 4 in R2878), so the falsifier holds there. The restarts that remained:
  - one on speed (0.22 m/s; the twin's speed gate is 0.15, the rig's 0.25);
  - two at 12-14 cm while the twin's 60 ms carriers were already diverging.
- **R2879 had one descent only,** too thin to judge.
- **It does not break a clean weld** (R2881).
- **Extra descent while over 8 cm:** at most 1.6 cm with the tip 20 cm above the plate, and 0.1 cm at the bottom.

## W4b (exploratory, 19:19-19:31; R2882-R2885)
The longer debounce, one run each against W4's 0.3 s. Every arm is scored only up to the ring first going below 0.9 m,
which keeps the 60 ms carriers' late sinking out.

| Arm (60 ms unless noted) | Runs | Descents | Restarts before the bottom | Bottom held | S (deg) / E (cm) | Contact |
|---|---|---|---|---|---|---|
| 0.3 s, 8 cm (W4) | R2878 / R2879 | 4 / 1 | 3 / 0 (1 speed, 2 position) | 7.9 / 17.6 s | 2.44 / 1.41, 4.89 / 5.17 | none |
| 1.0 s, 8 cm | R2882 | 1 | 0 | 24.4 s | 1.58, 1.93 | none |
| 1.0 s, 10 cm | R2883 / R2884 | 2 / 2 | 1 / 1, both speed | 7.4 / 10.7 s | 3.07 / 1.92, 5.97 / 6.50 | none |
| No delay, 1.0 s, 10 cm | R2885 | 1 | 0 | - | 0.59, 0.86 | welded 46.3 s |

**Verdict** [exploratory]
- **Fewer descents restarted before the bottom:** 2 of 5 at 1.0 s against 3 of 5 at 0.3 s (one run each, so thin).
  None of them was on position.
- **The restarts that remain are all on the twin's 0.15 m/s speed check.** The rig's check is 0.25 m/s.
- **At the bottom:**
  - the tip was more than 8-10 cm off the plate centre for at most 0.3 s at a time, so a 1.0 s debounce never ran
    out;
  - there was no off-centre contact, and no ring-tilt kick that stands out.
- **A clean weld still welds.**
