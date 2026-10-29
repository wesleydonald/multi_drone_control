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
