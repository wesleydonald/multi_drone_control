# Your timing fix in our twin: QUAD4 goes unstable when the twin has the rig's delay (29 Oct, 17:22; split added 17:50)

Short version: `mpc_command_execution:=partial_step` (Codex's elapsed-interval correction under oct10) is fine in the
twin with no added delay, but with 50-60 ms of command delay QUAD4 flipped in its climb in all 3 runs. Your current
send path climbed normally at every delay. Wesley has held the rig run of the fix until this is understood.

## What we flew
The twin of sq5_2 (29 Oct): three carriers in the triangle on plates 3/11/7, the 1.15 kg ring, QUAD4 joins plate 9 from
the floor with sq5_2's typed settings, oct10. The delay is added in every simulated Betaflight bridge (QUAD4's included)
from t = 25 s, after the ring's lift and before your TAKEOFF at ~26.3 s. 50 ms is the value calibrated on 22 Oct against
the rig's carriers.

| Send path | no added delay | +50 ms | +60 ms |
|---|---|---|---|
| Your current send (endpoint) | climb normal, welded (R2866, R2868) | climb normal (R2867, R2869) | climb normal (R2870, R2871) |
| The fix (partial_step) | climb normal, welded 46.7 s (R2874) | **flipped in the climb** (R2875) | **flipped in the climb** (R2872, R2873) |

The flip: from your TAKEOFF the body bounced vertically while the cable lifted off the floor (R2872: 0.10, 0.59, 0.19,
0.82 m), your throttle swung 0.29-0.57 at ~0.7-0.9 Hz (height peaks 1.4-1.6 s apart), then it diverged sideways and ended upside down (tilt 150-178 deg)
6-9 s after TAKEOFF.

Codex's runtime checks all pass in these runs: metadata reads oct10, `first_stage_partial_step`, stage 0.1 s; the first
command after TAKEOFF is +0.017 throttle over hover (+0.050 with your current send); the per-tick steps stay within the
oct10 rate bounds (approximately: your log stamps wall time and the twin runs on sim time).

## Which channel: both, each in its own phase (one run each, exploratory)
| The fix with 60 ms on | Climb | Approach | Outcome |
|---|---|---|---|
| all channels (R2872, R2873) | vertical bounce, flipped | not reached | upside down 6-9 s after TAKEOFF |
| the throttle only (R2876) | vertical bounce 0.10 -> 0.60 -> 0.16 -> 0.99 m at ~0.7 Hz, flipped at 29.5 s | not reached | upside down |
| the three rates only (R2877) | clean (max tilt 7 deg, to 2.15 m) | reached at 34.3 s; a ~0.7 Hz swing grew (tilt 6.5 -> 41 deg in 7 s) | fell and flipped at 45.5 s |
| your current send, all channels (R2870) | clean | reached | normal approach and descents |

So with partial_step, a 60 ms delay on either channel destabilises QUAD4 in this twin; your current send flies both
phases with the same delay.

## Our reading (not tested)
Your current send uses the first stage's endpoint, which leads the plan by ~67 ms. That lead happens to cover the loop's
delay on both the throttle and the rates. The fix removes the lead, and with a real delay the vertical loop (in the
climb, as the cable lifts off) and then the swing (in the approach) become unstable.

## Two more things you should know
- **After the flip your controller kept QUAD4 armed at idle (throttle ~0.05), upside down, until the run ended:**
  51-58 s in R2872/R2873, ~119 s in R2875. On our side this is by design: when your stream flies QUAD4 its fault is
  yours to handle, so our fleet does not land for it. On the rig the operator's DISARM covers it, but a tilt or crash
  check in your controller would too.
- **From the rig bags: the ring itself moves during your attach.** During sq5_2's attach the ring moved 4.6 cm rms in
  xy, 3.3 cm in height, and its tilt varied 2.2 deg (k1_4: 3.3 cm, 1.6 deg). In the twin it moves 0.1-1.1 cm. So on the
  rig plate 9 is a moving target, and part of QUAD4's ~6 cm error at the abort is the ring, not your swing.

## Questions for you and Codex
1. Does the reading above match what you'd expect from the oct10 OCP? Would you model the command delay in the MPC
   (a delayed initial state, or the plan advanced by the delay) instead of removing the lead?
2. Is there a version of the fix you'd want us to fly in the twin first (for example partial_step only after the climb)?

Files: the card `docs/experiments/2026-10-29_rig_timing_twin_attach.md`, the report `docs/reports/twin_timing_2026-10-30.md`,
the run folders `results/2026-10-29/R2866_*` to `R2875_*` (your MPC logs under `his_mpc/`).
