# Mis-seed robustness of the attach-during-circle demo (E10, 2026-09-10)

World truth: 0.6 kg ring on 0.5 m rods, tethers at 3/12/9 o'clock, newcomer at 6.
Controller belief varied one parameter at a time; everything else = attach_circle_n3_fast
(velocity mode + common-mode trim, settle 65 deg, hand-out 8 s, hold 10 s). Two repeats each.

| belief | runs | outcome | pre-weld tilt | tilt / height after resume | sweep RMSE |
|---|---|---|---|---|---|
| mass 0.45 kg (−25 %) | R0215, R0216 | **clean 2/2** | 33.7° | 2–7° / 0.60 m | 0.14–0.16 m |
| mass 0.60 kg (true) | R0208, R0209 | clean 2/2 | 34.5° | 3–10° / 0.60 m | ~0.11 m |
| mass 0.75 kg (+25 %) | R0217, R0218 | **abort 2/2** ~9 s after the weld | 28.7° | load lifted to 0.96 m, 30° | 0.37 m |
| cable 0.45 m (−10 %) | R0219, R0220 | **clean 2/2** | 35.1° | 2.6° / 0.60 m | 0.15–0.16 m |
| cable 0.55 m (+10 %) | R0221, R0222 | **abort 2/2** ~8 s after the weld | 37.1° | load dropped to 0.17 m | 0.43 m |

Reading: the envelope is **asymmetric**. Under-estimating the load or the cable is absorbed
(the common-mode trim supplies the missing lift; short-cable references sit inside the
rods' reach). Over-estimating either is fatal within ~10 s of the weld: a heavier belief
over-lifts the load (0.96 m) and tilts it; a longer belief places references the rigid
rods cannot reach and the load drops. Quan et al. report tolerance to ±25 % payload and
40 % cable uncertainty; on this system the safe side is the low side.

Rig rule that follows: seed `load_mass` at or below the measured value and measure the
rods short rather than long; never round either up.
