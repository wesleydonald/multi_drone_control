# Next lab day: DRAFT (written 15-16 Oct night; tree overnight-1015 until merged)

Fill in once known: the date and slot (early morning, the noise rule), whether four quads are approved, which cables
and discs are fitted. Plates count clockwise from the ring's +x (`docs/rig/plate_numbering.md`): carriers on 3 / 7 /
11 (typed `90,330,210`, any carrier on any of the three), QUAD4 on 9. Card for the attach flights: the 15 Oct card
(`docs/experiments/2026-10-15_attach_comotion_rig.md`) unless the hardware changes its bars (then a new card first).

## Setup (every terminal)

```bash
alias w5='source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 PYTHONPATH=$HOME/multi_drone_control/tools/lean_rclpy:$PYTHONPATH'
w5
git log --oneline -1
```

Before anything flies: packs charged, props checked, ModemManager stopped only if an FC goes on USB; QUAD4's flash
erased (download it after every flight). Lay the carriers straight out from plates 3 / 7 / 11 at 0.70 m (0.55 m rods)
or about 1.2 m (1 m cables). T2's `slot error` line must be under 10 deg; a red `SLOT OFFSET` means fix the layout
and relaunch.

## Part A: three quads (no QUAD4; allowed under the noise rule)

The twin flew all of this cleanly (R1723-R1752, `docs/analysis/traj3_2026-10-15.md`: path error 7-19 mm, ring tilt
under 3.1 deg except the spin at 0.3 m/s). Fly one shape per flight, each speed only after the slower one was clean.

```bash
ros2 launch bringup real_io_launch.py num_drones:=3 magnet_initial:=ON                                  # T1
T="tools/rig_flight.sh real_control_launch.py num_drones:=3 attach_azimuths_deg:=90,330,210 target_z:=1.0 lift_ramp_vel:=0.15"
$T load_traj:=circle traj_speed:=0.125 c125      # T2; then fig_8 (f125), line_x (l125), orbit (o125), spin (s125)
```
With 1 m cables add `cable_len:=1.0` (the twin: `traj3_L100_*`, see the morning report) and hover once first (`h3`).
LAND when it hovers again (circle, fig_8, spin) or after one or two laps (line_x, orbit). RViz panel: ARM, TAKEOFF,
LAND. Speeds 0.2 then 0.3 m/s only on a shape that flew clean at 0.125.

## Part B: the attach (four quads, only with the approval)

The 15 Oct sheet's setup block, then these differences (all Wesley's call; the morning report has the evidence):

| Item | 15 Oct | Proposed | Why |
|---|---|---|---|
| Aim | B (`trim_speed_gate:=0.20`) | B | descents 1.8-3.8 cm median (a1) |
| Catch radius `K` | 0.024 | 0.045 with 90 mm discs | 22 % -> 49 % of the bottom time inside (8 flights) |
| QUAD4 tether `L` | 0.575 | the measured length (1 m: `magnet_drop_below_quad`, `mpc_cable_length`, `L` all follow) | room from the carriers 0.45 -> 0.7 m; QUAD4 0.4 m higher over the ring; the twin attach passes with 1 m |
| Drop take-back | off ("GIVE UP after a drop", which does not work: card 2026-10-16) | **decided (Wesley 16 Oct):** `params_file:=$C/drop_takes_back.yaml` | a2_2; twin: 8 drop cases pass |
| Console | READY ignores the layout | **decided (Wesley 16 Oct):** `python3 tools/console.py --slot-gate` | a3 flew two plates off |
| his LAND (console button, new) | none | his planner's own LAND: straight down where QUAD4 is, magnet OFF | to land QUAD4 when it is clear of the ring; over a fallen ring it lands on it |
| QUAD4's mat | where it was | straight out from plate 9, 1.2 m or more from the ring centre | a1's tether passed 0.10 m from QUAD2 |

**The software change, one at a time (A/B, alternating flights), only if the go/no-go in the filter card holds and
Tejen agrees:** zero plate velocity, `attach_twist:=zero` in `partner_args`, against today's bridge (check the bridge's
start-up line names the arm; the tree must contain 9c04ee3: before it the switch did nothing in a lifted hover). Evidence: on the rig bags it opens a3_2's match check at 4.9 s (raw: never); twin: the
tip's bounce against the plate -27 % at 1.8 Hz, no worse at 0.8 Hz, no regression on a still ring. Draft bars: the
card (`docs/experiments/2026-10-16_plate_height_filter.md`, "Draft rig A/B"). The plate-height filter waits for his
bottom hold and proof lift to read the raw plate. His match hold (`match_dwell`) stays his 1.0 s.

The hardware rows (discs, tether) change the baseline itself: fly them first, then the A/B on top.

Order as on 15 Oct: props-off console check (c1), then the bench (b2, b2d) if the hardware changed, then attach
flights scored with `tools/proof_replay.py --catch` and `tools/attach_window.py` (`--catch` = the new K) and
`tools/match_gate_replay.py` between flights.

## Stop rules (unchanged, plus)

- A drop: with `drop_takes_back` on, nothing to press (QUAD4 is taken back and landed). Without it GIVE UP does not
  hand QUAD4 over either (card 2026-10-16): his LAND only once QUAD4 is clear of the fallen ring (it lands straight
  down: in the twin it put the tip on the fallen ring every time and, after a drop at the plate, left QUAD4 armed on
  top of it, R1844-R1846), or DISARM QUAD4 once it is low and clear of the ring.
- Two carrier releases in one session: stop and check every carrier magnet's hold (QUAD2 let go on 15 Oct at 21.9 V).
- QUAD4 hanging ~0.3 m above its mat on its own landing: DISARM it.
