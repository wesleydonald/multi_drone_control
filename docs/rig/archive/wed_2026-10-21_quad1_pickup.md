# Wed 21 Oct: QUAD1 pickup only, Tejen's controller (1 m cable, new 0.15 kg object, ID6)

His M1 IRL stack (his 19 Oct code, branch `tejen-sync2`), through `m1_irl_pickup.launch.py`. It uses his
`irl_commissioning.yaml` plus an overlay for QUAD1, the 1 m cable and the new object. The planner **holds after
the lift** with the magnet ON: no transit, no drop. ARM, TAKEOFF and LAND are yours. Our rig stack (`real_io`)
must not be running: it reads the same Motive stream.

## Setup (every terminal)

```bash
source ~/ros2_humble/install/setup.bash && cd ~/mdc_wt/tejen_sync2 && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23
```

In Motive, check that these bodies are tracked: QUAD1 = **11**, QUAD1's magnet marker = **M** (note its id), and
the object = **6**.

## 1. Measure (props OFF)

```bash
# T1
ros2 launch tejen_mission m1_irl_pickup.launch.py measure:=true magnet_body_id:=M
# T2: hold QUAD1 up with the cable hanging still          -> magnet_drop_below_quad
python3 tools/pickup_geometry.py
# T2: rest the magnet face flat on the object's centre     -> contact_height
python3 tools/pickup_geometry.py
```

Also measure the object's thickness `T` (m) with a ruler. Then Ctrl-C T1.

## 2. Fly

```bash
# T1 (check its [m1_irl_pickup] line: ids, /dev/QUAD1, the drop, the contact point, HOLD AFTER LIFT)
ros2 launch tejen_mission m1_irl_pickup.launch.py magnet_body_id:=M magnet_drop_below_quad:=<drop> contact_height:=<contact> object_thickness:=<T>
# T3
ros2 topic echo /join_planner/phase
# T2
ros2 topic pub --once /drone_command std_msgs/msg/String "{data: ARM}"
ros2 topic pub --once /drone_command std_msgs/msg/String "{data: TAKEOFF}"
ros2 topic pub --once /join_planner/land_now std_msgs/msg/Bool "{data: true}"     # LAND, after the hold
```

**Kill: keep this typed in T4:**

```bash
ros2 topic pub --once /drone_command std_msgs/msg/String "{data: DISARM}"
```

**Expected phases:**
1. TAKEOFF, then APPROACH_ABOVE_PICKUP: the body about 1.4 m up, with the magnet 0.35 m over the object.
2. SETTLE, then DESCEND_TO_PICKUP: a high tier, then a low tier, where the magnet turns ON.
3. MAGNET_ATTACH_WAIT, then LIFT_OBJECT: his proof needs the object up 2 cm, with the body up 6 cm and under 15 mm of
   drift.
4. **The hold:** the magnet 0.40 m above where the object was. Then LAND.

A miss or a stalled descent retries, up to 5 times. After that it holds with the magnet OFF: LAND.

## Stop rules

- **DISARM** on a swing that grows, a body drifting toward the cage, or the object knocked off the mat.
- **LAND** if the lift sags without lifting the object. His kT feedback may move only ±25 % from 22, at 0.5/s, and
  the 0.15 kg object is about 25 % of QUAD1's weight.
- **Before the first flight, props off:** ARM, then check that the magnet switches with
  `ros2 topic pub --once /magnet/command std_msgs/msg/String "{data: ON}"`. It goes back OFF within a second, because
  his planner re-sends its OFF. His radio drives aux channels 5-10 for the magnet; check that this changes nothing
  else on QUAD1's FC. DISARM afterwards.
