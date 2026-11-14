# Wed 4 Nov afternoon: what is still to do in the lab (props off, nothing flies)

Paste each T2 output back to the lab chat; the values go into Thursday's sheet.

```bash
alias w5='source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash && export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=23 MDC_LEAN_RCLPY=1 MDC_PLANNER_CPUS=0-7 PYTHONPATH=$HOME/multi_drone_control/tools/lean_rclpy:$PYTHONPATH'
```

## 1. Re-measure W, X, Y and L (needed: body 24 was redefined)
This morning's W 0.0365, X 0.0039, Y 0.2271 were measured with the old body 24. The catch check aims at the marker, so
they must be measured again with the new one.
```bash
tools/t1_four_markers.sh detach:=true                    # T1
w5; python3 tools/payload_geometry.py --seconds 2        # T2
```
- QUAD4's magnet caught **centred on plate 9**: T2 gives drone_3's **X, Y** and its height above the pivot, **W**. It must
  say plate 9, radius ~0.23.
- Then hold QUAD4 up with the cable **taut and still**, run T2 again: body-to-marker is **L**.
- Stop T1 afterwards.

## 2. The small object's contact height (needed for the pickup)
QUAD4's magnet face resting flat on the centre of the 54 mm object, props off:
```bash
w5; ros2 launch tejen_mission m1_irl_pickup.launch.py measure:=true quad_body_id:=14 magnet_body_id:=24 pickup_body_id:=6   # T1 (mocap only, no radio)
w5; python3 tools/pickup_geometry.py                                                                                  # T2
```
Paste `contact_height`. Stop T1 after.

## 3. If there is time
- Weigh the small object to confirm the repo's 0.060 kg (Te-Jen's commissioning puck, if it is the same one).
- Photos for the thesis, plain background, a ruler in every close-up:
  1. the whole setup, wide from a high corner, cameras visible, ring and quads in place (a tape arrow on +x);
  2. the square layout from the same corner, if set up;
  3. one quad close, side on: markers, cable attachment, cable, magnet;
  4. QUAD4's magnet close, with its marker;
  5. the magnet resting on an M2A plate, side and top;
  6. the M2A ring from directly above, all 12 plates, plate 0 identifiable (the 1.15 kg ring too);
  7. a ring plate with its cable attached;
  8. both pickup objects side by side, then each with its markers;
  9. the net in the ring, from the side and from above;
  10. the ground station (laptop with RViz and the console, radios, the Motive PC);
  11. a Motive screenshot with the rigid bodies streaming.

Done already (4 Nov): body 6 defined, body 24 fixed, packs charged, magnet face 20 mm, plates 55 mm, net 5 cm below the
ring's bottom when loaded, small object 54 x 22 mm.
