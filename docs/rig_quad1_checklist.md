# QUAD1 arming check (Wed 14 Oct, 09:00 setup, props off)

8 Oct: QUAD1 reported `!ERR*` from boot and refused ARM until a battery replug (r300, twice; r303 needed one too).
In Betaflight's CRSF flight-mode telemetry `!ERR` means arming is disabled and `*` means not armed, so the cause is
one of the arming-disable flags. Nothing was logged on our side, so the check below captures it in Betaflight's words.

| # | Step | Record |
|---|---|---|
| 1 | Plug QUAD1 into the laptop (USB), Betaflight Configurator, CLI: `status` with the battery in, drone on its plate, rod on the ring, untouched for 10 s | the line "Arming disable flags: ..." |
| 2 | Same, but plug the battery in by hand the way it is done on the rig (drone moved while it boots) | flags again (CALIB = gyro calibration waited for stillness; ANGLE = tilted past small_angle) |
| 3 | `get small_angle` on QUAD1 and QUAD2 | both values (25 is the default) |
| 4 | On the plate, read the Setup tab roll/pitch of QUAD1 resting with its rod on the ring | angle (if > small_angle: ANGLE is the cause; the rod or a leg on the ring edge) |
| 5 | `diff all` on QUAD1 and QUAD2, save both to `results/rig/2026-10-14/quad1_diff.txt`, `quad2_diff.txt` | lines that differ (aux/arm switch range, small_angle, gyro_calib settings, rx, failsafe) |
| 6 | Receiver tab with the T1 radio node up: channel 5 (arm) low at boot, throttle at its minimum | values (ARMSWITCH / THROTTLE flags if not) |
| 7 | T1 up, `ARM` from T3 with props off, three times, with a replug between only if it fails | armed yes/no each time, and the flags on any failure |

**If a flag is found:**
- CALIB or ANGLE: rest the drone level and untouched for 5 s after the battery goes in. Add that to the sheet's replug
  line, rather than changing small_angle (a Betaflight setting: Wesley's call).
- ARMSWITCH or THROTTLE: compare the channel map with QUAD2 (step 5).
- RXLOSS: rebind QUAD1's receiver, then check the link quality in T1.
