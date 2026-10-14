# Request: flight-controller settings on QUAD1, QUAD2, QUAD3 (and QUAD4 with Tejen)

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
1. While the drone shows `!ERR`, run `status` and note the arming-disable flags (we expect ANGLE).
2. Calibrate the accelerometer on a level surface (Setup tab, "Calibrate Accelerometer").
3. Run these and keep the output:
   - `get small_angle`
   - `get gyro_cal_on_first_arm`
   - `get acc_calibration`
   - `get align_board_roll`, `get align_board_pitch`, `get align_board_yaw`
4. Then:
```
set small_angle = 180
set gyro_cal_on_first_arm = ON
save
```
- `small_angle = 180` lets it arm at any tilt. They sit tilted on their rods and fly rate mode, so this is safe for
  us.
- `gyro_cal_on_first_arm` re-measures the gyro bias at the arm, when the drone sits still on its plate.

## Acceptance
Leave the drone on its plate for 10 min, then arm and disarm. Repeat 5 times per drone with no refusal.

## For QUAD4
QUAD4 is Tejen's. The same settings would help (it also showed `!ERR*` while waiting), but only with his OK.
