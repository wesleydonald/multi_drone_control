# For Tejen: what changed overnight on your side of tomorrow's (Fri 30 Oct) rig sheet (02:15)

Nothing here has flown on the rig. All of it is off unless typed, and logged in src/tejen_mission/CHANGES_FROM_TEJEN.md.

## 1. Your C++ planner on the low route: it stalls from outside plate 9's gap (twin)
Wesley wants QUAD4 never to fly over the carriers. It now comes in between them, 0.30 m over the plates, and our rig
launch refuses any planned QUAD4 body height over 2.6 m. With your C++ transit on that low route, from a start outside
plate 9's gap (QUAD4's mat at (-1, -1); the plate beside the plate-11 carrier), your planner:
- took authority (attach 38.5 s, pickup 56.4 s);
- then held for 140 s: your final collision check failed 103 / 104 times on one carrier's tether model (plate 3 for the
  attach, plate 11 for the pickup), although the real cable-to-cable clearance stayed 0.75 / 0.70 m.

Twin runs R2903 and R2904. Tomorrow's sheet therefore flies your direct (Python) transfer from a start in plate 9's gap.
Question: is the tether model's margin (swing allowance, tracking inflation) what blocks it, and would you loosen it?

## 2. Your descent restart and give-up, as typed tomorrow
- The descent's position restart at 10 cm, held 1.0 s on fresh tip samples (`descent_abort_xy`, `descent_debounce`).
  A repeated (frozen) tip pose earns no time. The speed restart is unchanged.
- `contact_give_up_s:=120` (your 45 s fits ~4 attempts at the rig's cycle times).
- Wesley chose sq6b (10 cm, 1.0 s): "attempt as much as possible".

## 3. The drop-off
- **Low route:** the object's bottom 0.30 m over the plates, and a 3 cm climb after the drop. Your MPC overshot
  0.18-0.19 m on losing the 0.15 kg plate in the twin (R2898/R2899), so the climb is short.
- **Your contact point on the real object** (typed `pickup_contact_height`, measured with your m1_irl_pickup measure
  mode). Without it, the combined launch flies your commissioning puck's geometry: 86 x 86 x 22 mm, +0.012 over its
  marker, against -0.0045 measured on 21 Oct.
- **Fallback k1_7:** your `delivery_rejoin`, QUAD4 taking off with the object on its magnet (`attach_magnet_initial:=ON`
  keeps the coil ON before your nodes start). Twin R2905: dropped into the net at 43.2 s.

## 4. The frozen magnet marker reaches your MPC on the rig, not in the twin
In the twin your MPC reads `/tejen/pendulum_swing_state`, which is computed from Gazebo's poses, so our freeze emulation
could not test you. On the rig the swing comes from the mocap marker, which froze 13-34 % of the time on 29 Oct.
Wesley is adding markers to it tomorrow.
