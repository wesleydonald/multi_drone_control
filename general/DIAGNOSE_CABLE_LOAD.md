# Cable-load controller — diagnosis runbook

Run these in order. Each step isolates one layer. Stop at the first one that fails
and paste its output — that pinpoints the fault.

Every terminal must first:
```bash
cd ~/multi_drone_control && source install/setup.bash
```

---

## STEP 0 — Planner math (offline, no sim, no ROS)
```bash
python3 -m controller_load_mpc.diag_equilibrium --cable-len 1.0 --elev 45 --hover-z 0.6
```
PASS: `OCP solve status: 0`, `t=1.850`, every `|ref-consistent|=0.0000`,
FF `throttle≈0.321 tilt≈10.3deg`.
(FAIL here = planner geometry/OCP is wrong; nothing else matters until this passes.)

---

## STEP 1 — Clean slate (kills zombie processes that corrupt every run)
```bash
./preflight_quad_load.sh
```
Must end with `clean — no stale processes.`

---

## STEP 2 — Launch the HOLD test (decisive)
No lift — the system's only job is to hold the taut config it spawned in.

Terminal A (Gazebo):
```bash
cd simulation_assets && gz sim three_soft_paper.sdf -v 4 -r
```
Terminal B (stack, HOLD mode):
```bash
ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py lift_ramp_vel:=0.0
```

---

## STEP 3 — Verify exactly ONE planner is publishing (contamination check)
Terminal C, BEFORE arming:
```bash
ros2 topic info /drone_0/reference_trajectory
```
Must say `Publisher count: 1`. If it says 2 → a zombie is back → Ctrl-C everything,
`./preflight_quad_load.sh`, restart from STEP 2. Do NOT trust a 2-publisher run.

---

## STEP 4 — Arm + take off
Terminal C:
```bash
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
```

---

## STEP 5 — Read the diagnosis (watch Terminal B)

Tracker line:
```
[diag dN] z=.. ez=.. exy=.. rdrift=.. thr=.. roll=.. pitch=.. |aT|=.. |aC|=..
```
The one number that matters here is **`rdrift`** (drone radius − reference radius;
`+` = flying outward, `−` = pulled inward):

- **`rdrift` stays ≈ 0, `exy` < 0.05, roll/pitch steady** → the tracker CAN hold.
  The fault was the lift transient, not the tracker → move to the lift test:
  re-run STEP 1–4 with `lift_ramp_vel:=0.03 target_z:=0.3` instead of `lift_ramp_vel:=0.0`.

- **`rdrift` grows positive (flies outward)** while holding a FIXED reference →
  the fault is in the tracker's horizontal/attitude control, not the planner.
  This is the answer we want — paste ~5 s of the log.

Let this run ~10 s and copy the `[diag]` block. That single controlled run tells us
reference-problem vs tracker-problem.

---

## One-shot capture (optional): save a clean log to a file
After STEP 4, in Terminal C:
```bash
ros2 topic echo /drone_0/reference_trajectory --once   # confirm reference contents
```
To capture the diag stream for pasting, just copy Terminal B, or launch B as:
```bash
ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py \
    lift_ramp_vel:=0.0 2>&1 | tee /tmp/hold_test.log
```
then `grep '\[diag' /tmp/hold_test.log | tail -40`.
