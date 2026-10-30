# Request: calibrate the accelerometer on QUAD1 and QUAD2 (Wesley, 30 Oct 2026)

**Problem.** QUAD1 and QUAD2 often refuse to arm (`!ERR*`), for example 2 of 3 ARMs in h4n on 30 Oct. Replugging the
battery helps for about a minute.

**Cause.** Their FCs believe they are tilted 25-112 deg while they sit at 12-15 deg (QUAD3 and QUAD4 read the real
tilt). Betaflight refuses to arm above `small_angle` (25 deg). On the bench QUAD1's accelerometer reads 1.10 g right
side up and 0.87 g upside down, so Betaflight (which trusts it only between 0.9 and 1.1 g) ignores it, and the
attitude drifts with the gyro.

**Please, on QUAD1 and QUAD2:**
1. Back up the settings: `diff all` in the CLI, saved as `docs/rig/fc/QUADn_diff_<date>.txt`.
2. Props off, the quad right side up on a level surface, untouched. Setup tab, "Calibrate Accelerometer"; it takes
   ~2 s and is stored on the FC (no `save` needed). Then the Setup tab reads roll and pitch ~0 and the Sensors tab's
   accelerometer reads X, Y ~0 (the web app shows 1 g as ~0.25 on Z with QUAD1's Betaflight 4.3.1).
3. The arming angle off: Configuration tab, Arming, "Maximum ARM Angle" 180, Save and Reboot (CLI: `set small_angle
   = 180`, `save`). The quads fly in rate mode, so the FC's attitude only gates arming; our console still blocks a quad
   on its side from mocap. On all four quads; then run the console with `--small-angle 180`.
4. Back up again: `diff all` saved as `docs/rig/fc/QUADn_diff_<date>_after.txt`.

**Approved** by the supervisor 30 Oct ("Also find the minimum arming angle in the configuration and either turn that
off or increase it"); docs/decisions.md.

**Check.** `python3 tools/fc_probe.py armcheck /dev/ttyACM0 --tx=/dev/QUADn --seconds=600` with the quad on its rod
(read-only): the accelerometer reads 1.00 +- 0.03 g, the FC's tilt matches the real tilt for 10 min, and nothing
blocks arming. Then 5 arms in a row without a refusal.

Evidence: registry RIG-1029-h4, RIG-1030-h3t and
RIG-1030-h4n, and this file's git history.
