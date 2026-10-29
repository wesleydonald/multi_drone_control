# Rig review, Thu 29 Oct 2026: what to fix

Four quads on 1 m cables carrying the new 1.15 kg ring, then Tejen's attach (sq) and pickup (k). Registry rows
RIG-1029-bench to RIG-1029-k1_5-7; logs and bags in `results/rig/2026-10-29/`.

**Verdict:** the carriers now fly the ring (the triangle with the 5.3 Hz notch held it for over a minute). Tejen's
attach and pickup got as far as MATCH and the pickup hover, then stopped: on the 1 m cable QUAD4's magnet swings
around its target by more than his windows allow.

## What happened
| Runs | Setup | Result | Cause |
|---|---|---|---|
| h4 | 4 quads, no filter | plate 0 released at the top of the lift | the ring bent at 5.3 Hz (opposite plates in phase) with each tracker feeding its measured pull back |
| h4f, f4 | 2 Hz low-pass on the measured pull | held; figure-8 2.4 cm off the path | bending reduced by ~25 % |
| ch1, ch1b, ch1b_2 | chain, low-pass 1 then 2 stages | plate 0 released (mid-lift; 0.4 s after the T formed); ch1b_2 the T sank | 2 stages stopped the bending (0.07-0.09 m/s^2); plate 0's magnet; the T at the throttle cap |
| sq1 | Tejen's attach, the T (carriers 0/3/6) | dropped at 33.6 deg before he reached the ring | the T's two loaded quads at 0.85-0.9 throttle, ring stuck at 0.90 m |
| sq2_2, sq2_4 | the triangle 3/11/7, throttle 0.95 | dropped / landed early | thrust fixed (0.69-0.83); the 2-stage low-pass's ~120 ms lag swung the ring at 1.1 Hz |
| sq2_5 | triangle + 5.3 Hz notch, his high transit | ring held ~87 s; climb 3.1 m in 24 s, MATCH 12 s, 1 descent aborted | his MATCH gate (5 cm for 1 s unbroken) |
| sq3 | his C++ transit at 0.30 m | never left the top of the climb | his stationary hand-over: QUAD4 at 0.08-0.09 m/s against his 0.08 |
| sq4_5 | + rig hand-over 0.12/0.30, climb 0.3 m/s | climb 8.5 s, then WAITING_ABOVE_SEARCH_FLOOR for the whole flight | my typed floor 2.36 m with no margin: the ring rode 3 cm low, QUAD4 held at 2.33 m |
| sq5_2 | his Python transfer (C++ off) | climb 7.8 s, transit 6.8 s, MATCH 1.7 s; **6 of 6 descents aborted** | each abort was the 8 cm position limit, on one tick: 8.1-9.5 cm at the abort; never the speed limit (0.02-0.14 of 0.25 m/s) |
| k1_4 | pickup | hovered over the object 26 s, never went down | his capture window 2.5 cm, 0.07 m/s, 0.5 s: the magnet sat 4.1 cm (median) off the object, inside 2.5 cm 17 % of the time |

## Fixes, by owner

### Tejen (his code; the message below)
1. **The swinging magnet** (the root of 2-4). On a 1 m cable it swings ~4 cm (median) round its target, peaks 8-9.5 cm,
   in hover and in descent. His MPC models the cable at 0.997 m (correct). Can it damp the swing, or plan the final
   contact from the predicted tip?
2. **The descent abort** trips on one tick past 8 cm (`contact_abort_xy_threshold`, rig 0.08): debounce it (e.g.
   0.3 s), use a filtered or predicted tip, or size it from the measured swing. The new slots are ~100 x 85 mm, so a
   contact a few cm off still catches.
3. **The MATCH gate and the pickup capture window** are too tight for a 1 m cable: MATCH 5 cm / 1 s (rig) and
   capture 2.5 cm / 0.07 m/s / 0.5 s. The rig flies typed 8 cm, 0.5 s, zero twist for MATCH (passed in 1.7 s) and
   6 cm / 0.15 m/s for capture (not flown yet). His view on safe values.
4. **The C++ transit on the rig:**
   - its stationary hand-over bounds: his 0.08 m/s / 0.15 m (the rig now 0.12 / 0.30, as the twin);
   - WAITING_ABOVE_SEARCH_FLOOR waits for ever with no message to the planner: a timeout, or a climb to the floor;
   - his box model of the carriers' tethers refuses a low transit end unless the swing allowance is ~1 deg (capsule
     tethers would not);
   - his backend latches itself off for good on one bad input (`start_skew`, `tracking_mismatch`).
5. **Minor:** his MPC's `object_vehicle_mass_kg` is 0.55; QUAD4 is 0.583 kg with the 1 m cable. His package's tests:
   24 fail before and after our changes (M2 IRL contracts, RViz configs, lab YAMLs).

### Ours (software)
1. **The 5.3 Hz notch** (`cable_notch_hz`, rig default) has flown only on the triangle (sq2_5): fly it on four quads
   (h4) before trusting it in the chain.
2. **Formations' thrust:** the T and the pair are at or over the cap with the 1.15 kg ring on 1 m cables. A check
   before flying or releasing into a formation (each carrier's predicted throttle), and a chain whose survivors stay
   balanced (release plate 0 first, or a pair the triangle can become).
3. **The twin cannot show the swing** (rigid links, a locked decision of 9 Oct): MATCH took 1-3 s there and
   15 s-never on the rig. A slack or pendulum cable for QUAD4 in the twin would let us tune his windows offline.
4. **Our bridge's publish order** (`start_skew`): `commit_backdate` 0.04 is typed on the rig; the real fix is the
   order in his node (states before commitments) or ours.
5. **The rig ring rides ~3 cm low** in hover (f4 mean -3.1 cm, sq4_5 0.977 m): the height integral or the thrust map.
6. **Scorer gaps:** `tools/attach_clearance.py` misses aborts after LAND and the tilt after the weld.

### Hardware (Wesley, 29 Oct afternoon)
1. **Plate 0** released under load 3 times (h4, ch1, ch1b) at ~3-5 N while the others held up to ~9 N. Nothing is
   different about the plate: QUAD1's magnet is a bit weaker, and a replacement is coming.
2. **QUAD4's cable** is the same metal cable as the carriers': it bends but does not stretch once taut. So the magnet
   swings as a pendulum on a fixed 1 m length; that swing is what beat Tejen's windows.
3. **The ring** is metal sheets bolted on and prone to flexing (photo of the bottom to come): the 5.3 Hz saddle bending.

## Changes made today (all in the working copy, nothing committed)
Rig defaults: `cable_notch_hz` 5.3 (Q 1; `cable_lp_hz` 0), his hand-over bounds 0.12 / 0.30 on the rig path. Typed
switches, off unless typed: `commit_backdate`, `tracking_check`, `climb_speed`, `cpp_authority`, `end_after_drop`,
`pickup_xy`, `pickup_speed`, `cable_lp_hz`/`cable_lp_order`; `geometry` passes through `partner_args`; the launch
refuses a typed search floor within 0.10 m of the climb end. Every change to his package is in
`src/tejen_mission/CHANGES_FROM_TEJEN.md`.

## Message for Tejen
See the reply in the session (also below), to send as it is or trimmed.

---

Hi Tejen, here's what we found on the rig today (29 Oct: 4 quads on 1 m cables, the new 1.15 kg ring with ~100 x 85 mm
slots; QUAD4 on its 1 m cable flown by your controller). Logs and bags if you want them: sq2_5, sq5_2, k1_4.

The short version: your transit and MATCH work on the rig, but every final step failed the same way. On the 1 m cable
the magnet swings around its target, a median ~4 cm with peaks of 8-9.5 cm, and your windows are tighter than that.

1. Attach descent (sq5_2): climb 7.8 s, transit 6.8 s, MATCH 1.7 s, then 6 of 6 descents aborted with "loss of
   relative alignment". Each abort was the position limit, 8 cm on the rig, on a single tick: the magnet was 8.1-9.5 cm
   off at the abort, never the speed limit (QUAD4 at 0.02-0.14 m/s of 0.25). Could the abort be debounced (say 0.3 s),
   or use a filtered or predicted tip? The new slots catch a few cm off-centre.
2. Pickup (k1_4): it hovered over the object for 26 s and never went down. Your tiered capture needs 2.5 cm, 0.07 m/s,
   0.5 s; the magnet sat 4.1 cm (median) off, inside 2.5 cm 17 % of the time. We've typed 6 cm and 0.15 m/s for the
   next try. Are those safe for your descent?
3. MATCH: 5 cm for 1 s unbroken took 12 s (sq2_5). 8 cm, 0.5 s and a zero plate velocity passed in 1.7 s.
4. The swing itself: your MPC has the cable at 0.997 m. Can it damp the magnet's swing, or time the contact to it?
5. Your C++ transit on the rig, three things that stopped it before it started:
   - the stationary hand-over: QUAD4 hovered at 0.08-0.09 m/s against your 0.08 bound (we now pass 0.12 m/s and
     0.30 m on the rig, as in our sim);
   - WAITING_ABOVE_SEARCH_FLOOR: with QUAD4 a few cm under the floor it waited the whole flight; a timeout or a climb
     to the floor would help (that one was my floor);
   - your box model of the carriers' tethers refuses a low transit end unless the swing allowance is ~1 deg.
   For the last runs we flew your Python transfer instead (attach_cpp_transit false).
6. Small: your backend switches itself off for good on one start_skew or tracking_mismatch (the start_skew comes from
   our bridge publishing commitments before states; we start them 40 ms early now). Your MPC's vehicle mass is 0.55;
   QUAD4 is 0.583 kg with the 1 m cable.

Everything we changed in your package is listed in CHANGES_FROM_TEJEN.md. All of it is off unless typed, except the
rig hand-over bounds above. Happy to call through any of it.
