# The rig-only thrust swing after a release: what drives it (thesis plan P6, finding 6)

12 Oct 2026, desk analysis from the logs on disk (no bag, no flight). Made by `tools/counter_swing.py`; the tables are
in `results/analysis/counter_swing_2026-10-12/table.md` and `table.csv`. Numbers below are from those files.

## The question

P6's double-count test (3_doublecount, 12 Oct) found that on the rig the survivors' commanded thrust moved against the
planned tension step when the resized plan reached the trackers, in 9 of 9 survivors (0.14 to 1.40 N), and then swung
at about 1 Hz; the twin's survivors did neither. The candidate left untested there: the resized plan lowers the
survivor's reference 4-5.6 cm in 0.35 s and the measured term reads that motion back as rod pull. This note asks what
the swing is made of, where it comes from, and what would remove it.

## How the measured pull enters the thrust (tracker, `cable_source mocap`)

`tracker_node.measured_cable_accel` with `_mocap_accel`: the rod's pull on the drone is the mocap acceleration (the
least-squares slope of the mocap velocity over 80 ms, then a 50 ms low-pass) minus the commanded thrust rotated into
the world (the same filter), plus gravity and drag. It is capped (12 m/s^2), slew-limited, and at node 0 it replaces the
planned pull: `ref_cable = planned + (measured - planned[0])`. So the commanded thrust is, to a close approximation,
the hover thrust plus the measured vertical pull plus the position loop's correction, and anything in the measured
pull is in the thrust one tick later. The plan's tension step never enters node 0 on this pull source (freeze 1.5, the
pure k = 1 model).

## What the logs show

Per survivor, the measured vertical pull fz = -m acz and the commanded thrust m kT u2; windows: 7 to 0.5 s before the
event, and the 5 s after the resized plan's arrival; for the carry flight, 8-28 s into the orbit.

**1. The thrust swing is the measured pull swing.** In every window of every rig flight the thrust follows fz with a
lag of 0 to 0.02 s and a correlation of 0.95-0.99 (table, `thrust_vs_fz`). What is left after subtracting fz (the
hover thrust and the position loop) swings 2-4 times less than fz (`resid_amp_N` 0.02-0.15 N against `fz_amp_N`
0.04-0.38 N).

**2. The measured pull swings before any release, on the rig only.** In the steady circle of 7 Oct (r206e60, no event)
fz swings 0.04-0.09 N at 1.45-1.65 Hz on all four drones, with the ring's tilt vector swinging 0.5 deg at 1.45 Hz. In
the 8 Oct pre-windows fz swings 0.04-0.31 N at the ring's own frequency (1.7 Hz on r303_4 with the ring at 1.2 deg;
0.3-0.8 Hz on r304_3 and r305). The twin's survivors read 0.00-0.01 N in the same windows with the ring at 0.04-0.10
deg: the twin has no such mode (rigid links, no mocap noise, no latency).

**3. The release excites that mode and the measurement reports it.** After the plan's arrival the ring's tilt swing
grows to 2.3-3.7 deg (hover) and 3.0 deg (circle), its dominant frequency 1.2-1.6 Hz, and fz swings 0.10-0.38 N with
it. The swing decays: fz half-range in 1 s bins from the arrival, r303_4 drone 0: 0.88, 0.31, 0.37, 0.23, 0.32, 0.33 N
(pre-window 0.35); r304_3 drone 1: 0.77, 0.26, 0.44, 0.46, 0.09, 0.38 (pre 0.34); r302_3 drone 2: 1.60, 0.80, 0.28,
0.20, 0.11, 0.09 (pre 0.28). The ring's tilt deviation from its pre-mean falls the same way (r304_3: 15.3, 6.1, 5.1,
4.1, 2.6, 2.5 deg). Nothing grows: the loop through the measurement is not unstable, it is a lightly damped physical
transient fed forward at gain 1.

**4. The "against the step" sign at the arrival is the measurement's phase, not a reference effect.** The thrust step
across the plan's arrival splits into the measured pull's step and the rest (position loop):

| flight | survivor | plan step (N, vertical) | thrust step at the arrival (N) | of which fz | of which the rest |
|---|---|---|---|---|---|
| r303_4 | 1 (30 deg) | +1.11 | -0.35 | -0.18 | -0.17 |
| r303_4 | 2 (150 deg) | +1.11 | -0.69 | -0.42 | -0.27 |
| r303_4 | 0 (270 deg) | -0.38 | +1.12 | +0.85 | +0.28 |
| r304_3 | 1 | +1.70 | -1.07 | -0.51 | -0.56 |
| r304_3 | 2 | +1.55 | -0.14 | +0.09 | -0.24 |
| r304_3 | 0 | -1.11 | +1.20 | +0.78 | +0.42 |
| r305 | 1 | +1.07 | -0.16 | +0.07 | -0.23 |
| r305 | 2 | +1.12 | -0.47 | -0.22 | -0.25 |
| r305 | 0 | -0.44 | +1.40 | +1.03 | +0.37 |
| r302_3 (announced) | 1 | +0.93 | +0.18 | +0.14 | +0.03 |
| r302_3 | 2 | +0.95 | -0.43 | -0.41 | -0.01 |
| r302_3 | 0 | -0.45 | +0.37 | +0.28 | +0.09 |

On the unannounced releases the plan arrives 0.47-0.55 s after the magnet opened, while the ring is still falling toward
its dip (0.7-1.0 s after the event): the rods beside the gap are unloading and the opposite rod loading, and the thrust
reports exactly that. The rest (the reference change: the resized plan moves the survivors' references 3-11 cm
sideways and 1-4 cm down within 0.5 s) adds 0.2-0.6 N in the same direction. In the twin the same reference change
comes with fz steps of 0.02-0.11 N and thrust steps of 0.07-0.16 N: with no rocking, the arrival is quiet.

**5. What a filter on the measured offset would do.** A first-order low-pass on fz replayed on the logs: tau 0.3 s
leaves 0.08-0.21 N of the post-arrival swing (from 0.10-0.38 N) and passes 61-82 % of the release step within 1 s of the
magnet opening; tau 0.5 s leaves 0.05-0.17 N and passes 49-66 %. The twin loses nothing it has (0.01-0.02 N either way).

## Reading

- The swing is not a controller artefact of the resize. It is the rig's own ring-and-rods rocking mode (1.4-1.7 Hz in
  steady carry, 1.2-1.6 Hz on three drones after the release), excited by the release, measured by the mocap pull
  estimate and fed forward into the thrust at gain 1. The twin has no such mode, so it cannot show the swing, and no
  twin pair can test a fix for it.
- The counter-step at the plan's arrival is the phase of that transient at that instant plus the reference change; it
  is not a double count (P6 3 stands) and not the reference drop alone (the twin has the drop without the step).
- For T6: the rig's Delta-theta carries this mode. The no-event floor of 4.8 deg (hover) and 6.3 deg (path) and the
  sigma_pre of 1.0-2.3 deg are this rocking; the twin's 0.1-0.3 deg sigma_pre is its absence. The ledger's class
  factors (0.85-1.70) absorb some of it; the resolution rule's minimum detectable effect at n = 3 (4.5 deg hover, 7-9
  deg path) is set by it.
- The latency finding (P6 4: 20 ms command-to-rate delay, the tracker's model has a 0.12 s lag instead) sits in the
  attitude loop and does not appear in these numbers: the thrust tracks the measurement within one tick.

## Proposed fix (no code written; Wesley's call)

1. **Filter the measured offset, not the plan.** Low-pass `(ac_cond - ref_cable[0])` before it is added along the
   horizon, tau about 0.3 s (one new tracker parameter, default off until flown). It keeps what the measurement is for
   (the slow redistribution after a release, the rig's ~0.55 N inward bias, the thrust map's error) and removes the
   rocking band. Cost: the release step reaches the feedforward later (61-82 % within 1 s instead of all of it), so the
   dip on an unannounced release may grow by a centimetre or two; the twin cannot say. Where to test: on the rig, a
   one-variable pair on the carry circle (R2's line, filter on against off: fz and thrust swing amplitude, path error,
   height sd), before any detach pair.
2. **Alternative, cheaper to reason about: cap the measured offset's rate** (a slew on the offset of about 1 N/s) rather
   than its band. Same test.
3. **Leave the planner alone.** Neither the resize timing nor the double count is the driver; the reference change
   contributes 0.2-0.6 N once, at the arrival.

Whether the rocking itself is helped or hurt by the feedforward (the drone as a stiff anchor against a soft one) these
logs cannot say: the mode is present at the same frequency with and without an event, and only the rig pair above
would show a change in its decay.

## Caveats

- Four rig flights (three unannounced, one announced), one rig carry flight, six twin runs. The spectra are over 5 s
  windows (0.2 Hz resolution); "frequency" means the dominant bin.
- The decomposition thrust = fz + rest is the tracker's own structure (gain 1 at node 0 on `mocap`); it does not prove
  causality by itself. The lag (thrust after fz by 0-0.02 s, never the other way) and the no-event carry flight do.
- The low-pass replay filters the logged fz offline; the flown filter would also change what the ring does next.
