# Request: flight-controller settings on QUAD1, QUAD2, QUAD3 (and QUAD4)

**From Wesley, 14 Oct 2026.** Arming refusals cost 11 of 46 launches on 14 Oct (registry rows RIG-1014-*, "ARM FAILED
... FC says '!ERR*'").

## What happens
- The carriers rest tilted 11-13 deg on their rods while they wait to be armed.
- Their FC reports `!ERR*` (arming disabled) for minutes at a time, and arming is refused. Replugging the pack
  usually clears it for about a minute.
- QUAD1's own attitude telemetry read a roll of -60 to -84 deg while it sat at ~12 deg. Its attitude estimate is far
  off, which points at Betaflight's small-angle arming check (default 25 deg).
- In flight the FCs report `AIR` (airmode). Both our controller and Tejen's send rate commands through the
  Betaflight rate curve, so the FC's attitude estimate is not used in flight. It only gates arming.

## Please, per drone (Betaflight CLI)
1. While the drone shows `!ERR`, run `status` and note the arming-disable flags (we expect ANGLE). Do the same
   once while it shows `AIR*` after a refused arm (no disable flag there, so that refusal has another cause).
2. Calibrate the accelerometer on a level surface (Setup tab, "Calibrate Accelerometer").
3. Watch the attitude in the Configurator for 5 min on a level bench: a slow drift points at the estimator, a
   steady wrong angle at the accelerometer.
4. Run these and keep the output:
   - `get small_angle`
   - `get acc_calibration`
   - `get align_board_roll`, `get align_board_pitch`, `get align_board_yaw`
5. Then:
```
set small_angle = 180
save
```
- `small_angle = 180` lets it arm at any tilt. They sit tilted on their rods and fly rate mode, so this is safe for
  us. Our console then checks the real (mocap) tilt before ARM instead, so a tipped-over drone still cannot be
  armed.
- Not `gyro_cal_on_first_arm`: it would delay the arm past our software's confirm window.

## Acceptance
Leave the drone on its plate for 10 min, then arm and disarm. Repeat 5 times per drone with no refusal.

## For QUAD4
The same settings would help (it also showed `!ERR*` while waiting).

All four quads are Wesley's; any FC change needs the supervisor's OK first (Wesley, 15 Oct). Backups before any
change: `docs/rig/fc/QUADn_diff_<date>.txt` (QUAD4's of 15 Oct: Betaflight 4.3.1, T-Motor F7 AIO, blackbox on
the onboard flash, accelerometer calibrated, small_angle at its default 25).
