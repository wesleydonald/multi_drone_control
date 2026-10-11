# Wed 14 Oct, step 3: props-off load test on the rig laptop (go/no-go for Part C)

The full combined graph (both stacks, real radios, real Motive), props off, for 10 min. The F1 bars (rig card
2026-10-21_combined_m1_rig §5.1, frozen in the card) decide whether R11a flies today. The same session checks the Part C
launch lines (they refuse with W or L missing or wrong), the kill path, the coil gate and the mux.
Install: `~/mdc_wt/rig-wed`, pinned at 0628d85. Order: the refusals, the 10 min, K1 and the score, then K2 and K3.

## 0. Set up (after W5)
- Props off on all four, packs in (the radios and FCs must be powered). Ring at its start spot, carriers on plates
  3/11/7, QUAD4 on its mat, puck in its place. Motive streams 11-14, 8, 24 and 6.
- **Stop T1** (`rio`): mode:=attach starts its own radios and mocap, and rig_flight refuses while T1 is up. Close
  every week4 terminal.
- Three fresh terminals: T2 (launch), T3 (ours), T4 (his commands, on this laptop). Paste into each:
```bash
source ~/ros2_humble/install/setup.bash && source ~/mdc_wt/rig-wed/install/setup.bash && cd ~/mdc_wt/rig-wed
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23
export MDC_REPO_ROOT=$HOME/mdc_wt/rig-wed      # no longer required (the root lookup accepts a worktree since the fix), harmless
fleet() { python3 ~/mdc_wt/rig-wed/tools/fleet_reach.py --send "$1"; }
C=$HOME/mdc_wt/rig-wed/configs/rig
A="tools/rig_flight.sh real_control_launch.py mode:=attach target_z:=1.0 lift_ramp_vel:=0.15 enable_approach_mpc:=false mocap_magnet_tip_body_id:=24 params_file:=$C/attach_m1.yaml"
W=0.031; L=0.58; R=0.012      # PLACEHOLDERS: type the 09:00 record (item 3 W, item 4 L, item 7(b) R)
P=$(python3 -c "print(round($L-0.003,3))"); Q=$(python3 -c "print(round($L-0.04,3))")
X="partner:=true partner_contact:=true partner_nodes:=true weld_radius:=$R"
PA="magnet_drop_below_quad:=$L mpc_thrust_ratio:=22 contact_dz:=0.05"    # + drop_puck_wait_s:=0 if item 11 says so
```
- T3: `git log -1 --format=%h` prints `0628d85`. Then `ls c_generated_code_load_planner/*.so
  c_generated_code_quad_load/c_generated_code/*.so` lists 4 files (built 11 Oct). If not, run
  `python3 tools/prebuild_planner.py 2 3 4 --load-mass 0.86 --drone-mass 0.55 --cable-len 0.55`.

## 1. The Part C line refuses (T2, before any T2 launch; each takes ~5 s and starts nothing, no radio opens)
| # | line | T2 must print `[ERROR] [launch]: Caught exception in launch ...` with |
|---|---|---|
| r1 | `$A $X proof_taut_len:=$P attach_cable_len:=$Q partner_args:="$PA" lt_r1` (no W) | `type weld_point_z:=<the 09:00 record ...>` |
| r2 | `$A $X weld_point_z:=$W attach_cable_len:=$Q partner_args:="$PA" lt_r2` (no L) | `type proof_taut_len:=<the 09:00 record's body-to-tip minus 0.003>` |
| r3 | `$A $X proof_taut_len:=$P weld_point_z:=$W partner_args:="$PA" lt_r3` (no rod) | `type attach_cable_len:=<QUAD4's rod from its record>` |
| r4 | as r3 plus `attach_cable_len:=$(python3 -c "print(round($Q+0.02,3))")`, name lt_r4 | `... disagrees with the taut length ... by +2.0 cm (more than 1 cm)` |
| r5 | the full line below with `magnet_drop_below_quad:=$(python3 -c "print(round($L-0.025,3))")` in partner_args, name lt_r5 | `partner_args magnet_drop_below_quad ... by -2.5 cm (more than 1 cm)` |

Pass: five refusals, `rig_flight: bag ... closed` after each. About 2 min after each refusal the same terminal prints
`rig_flight: REFUSED: the fleet manager never answered ...`. That is left over from rig_flight's reach check; ignore it.
Never run a rig_flight line while a T2 stack is up: its clean_slate kills T2's nodes.

## 2. The load test (10 min from his TAKEOFF)
1. T2: `$A $X proof_taut_len:=$P weld_point_z:=$W attach_cable_len:=$Q partner_args:="$PA" lt`. Must print
   `[launch] COMBINED M1 (rig): contact radius $R m at weld_point_z +$W, proof taut length $P m`, then
   `his rig nodes in 5 s: ... contact_target_z:=$W magnet_drop_below_quad:=$L mpc_thrust_ratio:=22 contact_dz:=0.05`.
   No `process has died`. RViz opens.
2. T2 prints `Fleet manager up: 4 drones, waiting for ARM.` and `rig_flight: the fleet manager answers in this domain`
   on its own (fixed 11 Oct: the manager announces at startup; no LAND needed).
3. T3, read back (each must print the value on the right):
```bash
for p in "dissipative_planner newcomer_lifecycle" "dissipative_planner newcomer_fault_land" \
  "dissipative_planner departed_ground_idle" "dissipative_planner take_back_release_guard" \
  "dissipative_planner weld_point_z" "dissipative_planner attach_cable_len" "fleet_manager require_fc_armed" \
  "magnet_attachment_manager attach_radius" "magnet_attachment_manager object_z_offset" \
  "magnet_attachment_manager proof_taut_len" "magnet_attachment_manager contact_still" \
  "online_join_planner magnet_drop_below_quad" "online_join_planner final_attach_offset_z" \
  "online_join_planner drop_puck_wait_s" "tejen/tejen_mpc thrust_ratio"; do echo "$p: $(ros2 param get /$p)"; done
```
   True True True True, W, Q, True, R, W, P, True, L, 0.05, 2.0 (0 if typed), 22.0.
4. **Not before T2 has been up 2 min** (on the desk, the graph's first minute had a 0.3 s stall that tripped his MPC's
   0.25 s pose watchdog). T4 (his ARM): `ros2 topic pub -t 3 /tejen/drone_command std_msgs/msg/String "{data: ARM}"`.
   T2: `[tejen.tejen_mpc]: Drone armed via command`. QUAD4's motors idle, props off.
5. T3: `fleet arm`. T2: `ARM sequence complete: all 4 armed, ready for TAKEOFF.` All four FCs armed, props off.
   Now run the section 4 checks (all but the his-stream one), before his TAKEOFF, so the scored window stays free of
   `ros2` CLI calls.
6. T4 (his TAKEOFF): `ros2 topic pub -t 3 /tejen/drone_command std_msgs/msg/String "{data: TAKEOFF}"`. T2:
   `[tejen.tejen_mpc]: Takeoff requested`, then every ~2 s `[elrs_mux] the partner's throttle held at armed idle: our
   planner is not flying the ring yet`. That line is expected: the F3 hold keeps QUAD4 at idle. **Never `fleet takeoff`
   in this test.**
7. Start the clock. For the next 10 min type nothing in T3 or T4: no CLI calls, nothing else running on the laptop.
   In T2, watch for `disarm triggered - disarming drone` (his MPC's pose watchdog), `Pose timeout`,
   `External-reference fault latched` and `No message received for 0.1 seconds`. Any one fails F1 already: note the
   time and carry on to 10 min.
8. At 10 min, T3: `E=$(date +%s.%N); echo $E`. Then the his-stream check and our span (section 4), then K1 (section 3).
9. After K1's Ctrl-C, T3, the score:
```bash
python3 tools/load_check.py results/rig/2026-10-14/lt --log results/rig/2026-10-14/lt.log --flight-window --end $E \
  --mocap | tee results/rig/2026-10-14/lt_load_check.txt
```
   **Go:** the last line reads `F1 load bars: PASS` (exit 0). The window line must read `his TAKEOFF to the typed end`,
   about 600 s.
   - `FAIL` (exit 1): no combined flight today (card §5.1, §6).
   - `INCOMPLETE` (exit 2): fix the command.
   - With `--mocap`, a gap seen on every pose topic at once came from Motive or the mocap node, not from the
     controllers. It still counts.

## 3. Kill path (props off; record each on video)
- **K1** (the load-test launch, after `E`): RViz DISARM (space).
  - T2: `Fleet command received: 'DISARM'`, `EMERGENCY STOP broadcast to 4 drones: operator DISARM`, and
    `[elrs_mux] drone 3: FLEET ABORT LATCHED (operator DISARM) - forwarding DISARM only`.
  - Every motor stops, QUAD4's too, although his MPC still sends armed.
  - T3: `ros2 topic echo --once /drone_3/mux_state` reads `latched`; `ros2 topic echo --once /drone_3/fc_arm_state`
    reads disarmed.
  - Then Ctrl-C T2: `rig_flight: bag results/rig/2026-10-14/lt closed`. Score (2.9).
- **K2** (T2 `... lt_k2`, then 2.2, 2.4 and 2.5): his DISARM from T4:
  `ros2 topic pub -t 3 /tejen/drone_command std_msgs/msg/String "{data: DISARM}"`.
  - T2: `Drone disarmed via command - initiating shutdown`; QUAD4's motors stop; the carriers stay armed.
  - Then T3 `fleet disarm`: `fleet: DISARM sent`, the carriers stop, mux_state reads `latched`. Ctrl-C T2.
  - Not desk-tested: record how long QUAD4 takes to stop.
- **K3** (T2 `... lt_k3`, then 2.2, 2.4 and 2.5): kill his MPC alone. T3:
  `ps -eo pid,args | grep '[t]ejen_mpc/lib/tejen_mpc/main'`, then `kill -9 <that pid>`.
  - T2: `[main-..]: process has died [pid ..., exit code -9`. About 0.5 s later QUAD4's radio prints `No message received
    for 0.1 seconds. Disarming motors.`, QUAD4's motors stop, and `/drone_3/fc_arm_state` reads disarmed.
  - Our side does nothing: the carriers stay armed and mux_state stays `partner`. The silence takeover (card F1-new)
    is not built. Desk 11 Oct: no fleet reaction.
  - Then Ctrl-C T2 with the carriers armed: every motor stops (the radios die with T2). Bar: within 0.5 s, by video.
  - T3: `pgrep -af mdc_wt/rig-wed/install` prints nothing.

## 4. Coil gate and mux (between `fleet arm` and his TAKEOFF; the his-stream check and our span after `E`)
- **T2 at boot:**
  - `[magnet_gate] drone 3: /drone_3/magnet_coil follows his /tejen/magnet/command until the first ON on
    /drone_3/magnet_ours (our span), then ours only; his puck from /tejen/object_attached`;
  - `[magnet_gate] our OFF ignored before our span: the coil stays his (ball)`;
  - `[elrs_mux] drone 3: forwarding APPROACH until /magnet/object_attached (latch=True, require_live=True)`;
  - `Drone 4 mux: partner`.
- **One coil writer (F4):** `ros2 topic info -v /drone_3/magnet_coil` shows Publisher count 1 (magnet_gate_3) and
  /drone_3/elrs_interface among its subscribers. `ros2 topic info /drone_3/magnet` prints `Unknown topic` or 0
  subscriptions, so a W4/W5 `/drone_3/magnet OFF` from shell history cannot reach QUAD4's coil.
- **States:** `ros2 topic echo --once /drone_3/mux_state` and `/drone_3/magnet_gate` both read `partner`.
- **His stream (F3, O6):** `ros2 topic echo --once /drone_3/ELRSCommand_tejen` shows channel_2 above -1 after his
  TAKEOFF (desk 0.137) and channel_5/7 = 1.0. `/drone_3/ELRSCommand` shows armed true, channel_2 -1.0 and
  channel_5/7 = 0.0.
- **Our span (one-way; after `E`, just before K1):** `ros2 topic pub --once /magnet/command std_msgs/msg/String
  "{data: 'ON'}"`.
  - T2: `[magnet_gate] OURS from now (first ON from the ring-magnet manager): his commands are ignored`, then
    `coil ON (our command)`, then `his OFF ignored: the coil is ours since our span began`.
  - `/drone_3/magnet_gate` reads `ours`, and QUAD4's coil holds the puck to its face by hand.
  - K1 follows. The radio latch keeps channel 6 ON after DISARM. Note whether the coil still holds after DISARM and
    after Ctrl-C (receiver failsafe).

## What props-off cannot show
The in-flight load: the planner's lift and hover phases, and the trackers flying real references. Also the mux
releasing his throttle (`our planner flies the ring`), the real magnet catching on plate 9, the slack tether, the
proof lift, the hand-over and the take-back. Gaps measured here are a floor, not a ceiling.

## Desk dry runs, 11 Oct (fake_mocap 100 Hz, no radios, `require_fc_armed:=false`, rig_flight's bag)
Both bags are under `results/desk/2026-10-11/` in this worktree. Scored with `load_check.py --flight-window --end $E`.
- `lt_dry_2`: the whole graph at nice 19, 387 s window.
- `lt_norm`: normal priority (nice 0, through `systemd-run --user`), 601 s window, `fleet arm` inside the window, no CLI
  calls in it.

| topic (bar) | nice 19: max gap s | normal: max gap s | normal: p99 s | normal: verdict |
|---|---|---|---|---|
| /drone_3/ELRSCommand_tejen (0.15) | 0.358 | 0.095 | 0.049 | PASS |
| /drone_3/ELRSCommand (0.15) | 0.363 | 0.095 | 0.049 | PASS |
| /drone_3/ELRSCommand_diss (0.15) | 0.362 | 0.076 | 0.026 | PASS |
| /drone_0..2/ELRSCommand (0.15) | 0.220-0.236 | 0.069-0.083 | 0.027 | PASS |
| /join_planner/reference (0.30) | 0.355 | 0.077 | 0.051 | PASS |
| pose topics (no bar) | 0.341 | 0.018 | 0.013 | |
| fault, pose timeout, his disarm lines (0) | 1 | 0 | | PASS |

- **At normal priority the 10 min pass with margin.** The tracker loops' worst tick gap was 0.083-0.097 s (p99
  0.031-0.033 s). His MPC: work p99 18 ms, max 29 ms; loop period max 95 ms.
- **CPU, 60 s armed:** 712 % of 1200 %. The dissipative planner and his MPC 80 % each, the trackers 71-74 %, his
  planner 70 %, the ring bridge 49 %, the bag recorder 58 %.
- **Before the window**, about 40 s after T2 started, every stream stalled 0.32-0.37 s at once. That tripped his 0.25 s
  pose watchdog and disarmed QUAD4 on its mat (hence step 2.4's 2 min wait).
  - Not GC, swap, major faults or CPU: the machine was 48 % idle.
  - It coincided with a 16 MB disk-write burst.
  - It never recurred: ARM, TAKEOFF, `fleet arm` and CLI calls later were clean.
- **At nice 19 the stalls were starvation.** Each came while a `ros2` CLI command at a higher priority was starting.
  tracker_0's `Pose timeout (0.34s)` was its whole process not running for 0.346 s; its mocap had no gap.
