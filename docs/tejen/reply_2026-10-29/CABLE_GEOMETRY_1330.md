# QUAD4's cable geometry in sq5_2 and k1_4: swing, not a bend (29 Oct ~13:30, for Tejen)

Your follow-up asked to separate target error from the anchor-to-magnet geometry used for the swing. This is that
geometry, from the bags, independent of any target.

**Method.** Per phase window, at 30 Hz (sample-and-hold), from the bag's `/drone_3/motion_capture_state` (QUAD4's body
pose) and `/magnet_tip_pose` (marker 24), with the rig's pendulum math (`partner_rig_state.py`, your IRL convention):
anchor = body + R(q) (0, 0, -0.030), r = marker - anchor, phi = atan2(r_x, -r_z), theta = atan2(r_y, -r_z), length =
|r|. Each angle is split into its mean over the window and the oscillation about it. Your MPC's logged phi and theta
(`his_mpc/external_reference_*/log.csv`, its `timestamp` column) are averaged over the same windows. All times are this
laptop's wall clock (the rig runs `use_sim_time` false); the bag's stamps are receipt times. Script:
`cable_geometry.py` (next to this file in ~/Downloads; run `python3 cable_geometry.py <bag dir> <his mpc log.csv> <phase> ...`).

**Reference length.** Props-off with the cable held taut, body-to-marker was 1.0412 m (RIG-1029-bench). With the anchor
3 cm below the body, a straight taut cable gives anchor-to-marker ~1.011 m. A bent cable shortens that chord.

| run, window | anchor-to-magnet (median, 5-95 %) | mean phi / theta | oscillation phi / theta (rms) | dominant | mean lateral offset | oscillation (rms) | your MPC's phi / theta (mean, rms) |
|---|---|---|---|---|---|---|---|
| sq5_2, approach after the transit (6.8 s) | 1.009 m (1.006-1.019) | -0.42 / +0.14 deg | 1.88 / 1.85 deg | 0.73 Hz | 0.8 cm | 4.6 cm | -0.42 / +0.14, 1.88 / 1.89 |
| sq5_2, MATCH (1.7 s) | 1.009 (1.002-1.009) | -0.53 / -1.15 | 2.56 / 2.36 | 0.59 | 2.2 | 6.1 | -0.47 / -1.27, 2.61 / 2.26 |
| sq5_2, MATCH (1.1 s) | 1.009 (1.008-1.020) | -1.97 / -0.50 | 1.59 / 0.58 | 0.88 | 3.6 | 3.0 | -1.98 / -0.56, 1.50 / 0.72 |
| sq5_2, descent (1.8 s) | 1.009 (0.993-1.011) | -0.54 / +0.25 | 2.00 / 0.66 | 0.56 | 1.0 | 3.7 | -0.56 / +0.16, 2.01 / 0.70 |
| sq5_2, MATCH (2.6 s) | 1.009 (1.001-1.038) | -0.58 / -1.11 | 2.62 / 1.06 | 0.38 / 0.77 | 2.2 | 5.0 | -0.69 / -1.13, 2.57 / 1.05 |
| sq5_2, descent (1.9 s) | 1.009 (0.996-1.016) | -0.68 / -1.07 | 1.58 / 1.43 | 0.51 | 2.3 | 3.7 | -0.59 / -1.06, 1.66 / 1.38 |
| sq5_2, approach (3.2 s) | 1.009 (1.006-1.028) | -0.74 / +0.02 | 5.07 / 2.45 | 0.63 | 1.3 | 9.9 | -0.86 / +0.11, 5.03 / 2.41 |
| k1_4, SETTLE_ABOVE_PICKUP (26.3 s) | 1.008 (1.007-1.010) | -0.64 / +0.36 | 1.75 / 1.33 | 0.70 / 0.59 | 1.3 | 3.9 | -0.64 / +0.35, 1.76 / 1.35 |

(The windows shown are those of 1 s or more; the climbs, where the magnet leaves the floor, are left out.)

**Reading.**
1. **The cable is straight and taut in flight.** The chord is 1.008-1.009 m in every window, within ~3 mm of the
   props-off taut straight length. A bend large enough to move the magnet by centimetres would shorten it.
2. **Little mean offset, mostly swing.** The mean hangs within 0.5-2 deg of the anchor's vertical (about 1-3.6 cm).
   The oscillation about that mean is 1.6-5 deg rms (3-10 cm), at 0.5-0.9 Hz; a 1 m pendulum is ~0.5 Hz. The
   magnet-to-target errors (k1_4: median 4.1 cm, offset 1.4 cm, oscillation 5.1 cm rms) are this swing plus QUAD4's own
   body motion relative to the target.
3. **Your MPC saw this swing.** Its logged phi and theta match the angles rebuilt from the bag to ~0.1 deg in every
   window: the swing state it received was the measured one, not a bend-biased one.

**Limits.** A kink right at the anchor would leave the chord straight and show only as a mean angle. So any static bend
is bounded by the means above (at most ~2 deg, ~3.6 cm, in any window; ~0.5-1 deg in most), not excluded. QUAD4's yaw
stayed near zero, so the heading frame and the world frame coincide here and cannot separate a bend that turns with
the quad from a fixed bias. No cable shape was recorded.

**Correction (13:55, after Codex's review, DECISIONS_1355.md section 4).** Reading 1 overstates the chord test: chord
shortening is quadratic in the bend, so a symmetric 4 cm midpoint bow on a 1 m cable shortens the chord by only ~3.2 mm.
Millimetre agreement with the bench chord therefore cannot exclude a centimetre-scale bow. The mean angle is not a strict
bound on an unobserved bend either, if other offsets compensate it. And the MPC's agreement validates the signal
plumbing, not the physical geometry (both use the same poses and anchor transform). What stands: the oscillation is real
(long k1_4 window, 26 s), and a single large constant bend does not explain the target error.
