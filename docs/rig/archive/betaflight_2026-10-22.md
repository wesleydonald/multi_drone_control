# Betaflight and radio timing (QUAD3, read 22 Oct 2026)

Read-only CLI on QUAD3 over USB (FC serial 204236713432): `version`, `status`, `diff all`, `dump all`,
`get blackbox`, `rc_smoothing_info`, `tasks`, `flash_info`. Nothing was set, saved or erased. Raw output:
`results/rig/2026-10-22/betaflight/bf_quad3_readonly.txt`. Wesley expects QUAD1, QUAD2 and QUAD4 to carry the same
configuration (not read). The live radio-rate check was dropped: the radio module cannot be plugged into the laptop.

## What the FC runs

| Item | Setting | Meaning for us |
|---|---|---|
| Firmware, board | Betaflight 4.3.1, STM32F722 (TMOTORF7_AIO); MPU6500 gyro | |
| Flight mode | Modes: ARM on AUX1, USER2 on AUX4 (the magnet pin); no ANGLE mode | Rate (acro) mode: the FC holds body rates, the tracker closes the attitude loop through mocap |
| Rate map | Rate profile 3 active: ACTUAL, centre 100 and max 100 deg/s, expo 0 | Linear, full stick = 100 deg/s; the twin's emulator uses the same map |
| Rate PID | roll P45 I80 D40 F120; pitch P47 I84 D46 F125; yaw P45 I80 D0 F120 | Feedforward on: the FC leads the rate command |
| Loop and filters | gyro and PID 8 kHz; gyro LPF1 PT1 250-500 Hz dynamic, LPF2 PT1 500 Hz, 3 dynamic notches 150-600 Hz; D-term PT1 75-150 Hz and 150 Hz | About 1-2 ms of lag |
| RPM filter | 3 harmonics set, but bidirectional DShot is off | Inactive |
| Motors | DShot600, airmode, anti-gravity, throttle boost 5 at 15 Hz | Throttle goes to the motors with no FC loop on collective thrust |
| RC smoothing | ON, auto, factor 30 (throttle 30) | Cutoff about 0.375 x the received frame rate (the idle reading showed FF 38 Hz = 0.375 x 100) |
| Receiver | CRSF on UART4 | |
| Blackbox | Set to the onboard flash at 1/4 rate, but the flash chip is not detected (`flash_info` empty, 2 SPI devices) | No flight logs on QUAD3 |
| Failsafe | Stage 1 for 1.5 s, then DROP (motors off) | A radio loss over 1.5 s drops the quad |
| Arming | small_angle 25 deg; craft name unset | The bench read showed RXLOSS ANGLE CLI: radio off, tilted on the bench, in the CLI |

## Radio link

- The tracker publishes `ELRSCommand` at 50 Hz. `elrs_interface.py` writes the latest frame to the radio module every
  10 ms (921600 baud). It ignores the module's OpenTX-sync timing frames.
- ELRS RF mode in the link statistics (14 Oct logs, `print_telemetry` was on then): QUAD1-3 mode 8, QUAD4 mode 12.
  Read against ELRS 3.x's rate list, 8 is "333 Hz Full" (LoRa) and 12 is "F500" (FLRC 500 Hz). This is an inference
  from the mode number; the FC's own reading of the frame rate was not taken (radio off).

## Command-to-motor timing on the rig

| Stage | Estimate | In the twin? |
|---|---|---|
| Laptop frame hold (100 Hz writer after the 50 Hz tracker) | 0-10 ms, ~5 ms mean | No |
| ELRS packet wait and air time at 333 Hz | ~2-3 ms | No |
| RC smoothing (PT3 at ~125 Hz; 38 Hz at worst) | ~2 ms (6 ms at worst) | No |
| Gyro and D-term filters | ~1-2 ms | No |
| Radio and FC together | ~10-15 ms | |

## What it explains

The pair-swing design measured the loop from today's bags (`results/analysis/push_1022_designs.json`, package
pair-swing):

| Path | Rig | Twin (base) | Betaflight reading |
|---|---|---|---|
| Stick to body rate (attitude) | 17-24 ms | 49-57 ms | The rig is faster: feedforward F120-125 and a stiff 8 kHz rate loop; the twin's emulated loop is PI without feedforward (kp 0.5, ki 5) |
| Throttle to lift | 44-67 ms | ~15-25 ms | The rig is 25-45 ms slower: nothing on the FC closes a loop on collective thrust, so the motors' spin-up shows in full; the twin's motors have a 20 ms time constant |
| Body rate in the tracker's state | +28-33 ms | +4-5 ms | Not Betaflight: our mocap node's angular-rate EMA (alpha 0.2, always on) |

So the delayed twin's uniform ~50 ms command delay stands in for two different things: a slower throttle and a
lagged body-rate estimate. Per channel, the rig is slower on the throttle and faster on the attitude.

## Consequences

- No Betaflight change is needed for the timing work (FC changes need the supervisor's OK).
- The rig-timing twin (pair-swing round B) should delay the throttle only (or slow the twin's motors), not the rates.
  Its attitude path stays ~30 ms slower than the rig's unless the emulator gains Betaflight's feedforward.
- Cheap rig-side candidates in our code: write each frame on the radio's own timing instead of a free 100 Hz timer
  (~4 ms), and replace the body-rate EMA (round D4 / fix F3).
- Motive's own latency is not measured: the stream from the Motive PC carries only `id | x,y,z | qx,qy,qz,qw`, and the
  node stamps each pose on arrival. Measuring it needs the sender on the Motive PC to add its frame time.
