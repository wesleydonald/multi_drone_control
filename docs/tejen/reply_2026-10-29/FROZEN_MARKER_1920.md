# QUAD4's magnet marker freezes in flight, and what we built for the descent restart (29 Oct, 19:20)

For Tejen (and Codex). Found tonight in the 29 Oct bags (docs/analysis/rig_1029_replays.md). Nothing here is flown yet.

## 1. The magnet marker freezes ~0.2 s at a time
Motive repeats body 24's (QUAD4's magnet) last pose while it has lost the marker; the stamps keep advancing, then the
pose jumps. The ring and the quads' bodies do not do this.

| Flight | Tip samples repeated | Holds | Median hold | Jump after a hold (median, max) |
|---|---|---|---|---|
| sq5_2 (attach) | 13.4 % | 141 | 0.20 s | 1.4 cm, 15.9 cm |
| sq2_5 (attach) | 17.5 % | 203 | 0.21 s | 2.0 cm, 158 cm |
| k1_4 (pickup) | 2.7 % | 27 | 0.20 s | 2.2 cm, 6.0 cm |

What it touches:
- **Your descent restart:** 3 of sq5_2's 6 restarts came on the first new sample after a hold.
- **Your swing state:** your MPC's phi/theta repeat only 0.3 % of ticks, because they are rebuilt from QUAD4's moving
  body and the held tip. So during a hold the swing angle is wrong rather than frozen: about 1 deg of false swing for
  every 2 cm the body moves, then a step when the tip updates. Untested whether that feeds the swing.
- Our side: the co-motion proof reads the same pose.

Next on our side: check body 24's coverage over plate 9 in Motive before Friday's attach flights.

## 2. Why a 0.3 s debounce is not enough on the rig
Over sq5_2's attach the magnet was more than 8 cm off plate 9 for 46 % of the time, in stretches of 0.4-1.7 s. That is
the swing (tip 8.8 cm rms) plus the ring's own drift (plate 5.1 cm rms; our carriers, below 0.3 Hz).

Replayed on that recording: the share of start times from which a full 3.9 s descent (0.30 m at 0.08 m/s) would not
have been sent back up.

| Restart | 8 cm, one tick (as flown) | 8 cm, 0.3 s | 8 cm, 0.5 s | 8 cm, 1.0 s | 10 cm, 0.3 s | 10 cm, 1.0 s |
|---|---|---|---|---|---|---|
| Full descents that survive | 0 % | 2 % | 12 % | 49 % | 28 % | 85 % |

The recorded data cannot show what QUAD4 would have done after the real restarts.

## 3. What is in the fork (off unless typed)
Logged in CHANGES_FROM_TEJEN.md, 18:43 and 19:10.
- **`contact_abort_xy_debounce_s` (launch arg `descent_debounce`, s), Codex's section 2:**
  - the contact descent's position restart, and the bottom's when there is no bottom hold, waits for that long a
    continuous violation, timed on the tip samples;
  - a stale tip, a repeated (frozen) tip, or a gap over 0.10 s between new samples restarts the count;
  - each attempt counts afresh, and the descent's count carries into ATTACH_READY;
  - the 0.25 m/s speed restart is unchanged.
- **Launch arg `descent_abort_xy` (m):** the restart distance, typed only (our rig path 0.08 otherwise).

Twin (60 ms command delay, exploratory, R2878-R2881):
- With 0.3 s the single-tick restarts are gone.
- A clean no-delay run still welds (R2881, 46.2 s).
- At 50-60 ms the descents reach the bottom, but the twin's own contact gate (1 cm) never closes.
- 1.0 s and 10 cm are flying in the twin now.

## 4. The pickup
On k1_4 your capture predicate (xy, filtered face-object speed and clearance together for 0.5 s) never closed as flown,
and it was the speed term that held it, not xy. With 6 cm / 0.15 m/s it closes 0.56 s into the hover in replay. That is
the provisional above-object trial Codex called defensible, flown without the timing change.

## Questions
1. Would you rather the debounce be 1.0 s at 8 cm (49 % in replay) or 1.0 s at 10 cm (85 %), given the ~100 x 85 mm
   slots and your bottom hold's 12 cm gross limit?
2. Should your swing estimate skip a repeated tip pose (hold the last good angle rate instead)?
