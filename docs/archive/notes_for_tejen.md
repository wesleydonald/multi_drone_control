# Notes for Tejen (collected by Wesley's integration loop, 2026-09-28)

Things found in or about your code while integrating M1/M2. Nothing here was changed in your repo; fixes to our fork
of your code are listed in authority.md §6.

## Requests before the lab on Wed 30 Sep (draft for Wesley to send)
- Weights of quad1-4 with pack, rod and magnet, and the ring (we compute each drone's predicted carry throttle with
  tools/headroom.py; the tracker caps throttle at 0.6 and the rig free-hovers at about 0.45).
- A Betaflight `diff all` per quad: failsafe procedure and delay, rxfail on throttle and on the magnet channel (AUX4 /
  ch 6: does the magnet drop on a failsafe?), throttle_limit, motor_output_limit, TPA, thrust_linear, airmode, arming
  flags, runaway_takeoff, blackbox.
- Motive: which program forwards to UDP 1511, can it send the frame timestamp, the ring's body id (8 or 9), and whether
  the magnet-tip bodies 21-24 exist.
- Please do not run the udev install on our laptop (item 5); `ls -l /dev/QUAD*` should show all four.
- Your M1 radio node opens the first /dev/ttyUSB*: please pin serial_port so it cannot take one of ours.
- Magnet ownership and the ELRSCommand_tejen remap for combined runs (items 7, P14): to agree before any combined test.
- Two fixes from our fork for your tree: the mocap receive loop (item 1) and the join-stall padding (item 11).

1. **Mocap receive-loop bug (rig safety).** `drone_communication/motion_capture_publisher_irl.py` (used by
   `irl_state_pipeline.launch.py`, your M1 IRL) and `motion_capture_publisher_node.py`: `return None` inside the
   `while` receive loop ends the node's loop on ONE malformed packet (`'|'` missing or wrong part count); a non-UTF-8
   packet raises out of it too. Every pose then goes stale mid-flight. Fix: `continue` instead of `return None`, and wrap
   `data.decode()` in try/except UnicodeDecodeError -> continue. Your `m2_irl_mocap_router.py` is already fine.
2. **Sim Betaflight bridge rate estimate (M2 ring rock).** Your sim bridge differences unstamped mocap poses over an
   averaged arrival period; under CPU load the samples bunch and the rate loop sees spikes. On our M2 bench under load:
   pose-rate 3/6 runs rocked past 3 deg (one abort), gyro-rate 0/6. We now run the sim rate loop on the X3 gyro by
   default (`rate_source imu`, `M2_RATE_SOURCE=pose` restores the old path). Sim only; the real Betaflight uses its gyro.
3. **Tether length in M2.** Your MPC uses L = 0.531 m in M2 while the join geometry uses 0.475 + 0.04 = 0.515 m.
4. **Collision clearance.** In our M1 run R0687 your rod came 0.294 m from a carrier body (our metric bar 0.30).
5. **udev.** Your udev install command overwrites our rules file on the shared laptop and would leave only QUAD2/QUAD4.
6. **Drone id map.** Your drone_0 is quad2, ours is drone_0 = quad1 (for the combined rig launches we need one map).
7. **Magnets in a combined M2 rig graph.** If our graph owns the radios, your magnet commands no longer reach the magnets
   (our radio latch is ON from boot through your join); if your elrs_interface_irl owns them, ours cannot reach them. We
   need to agree who owns each drone's magnet channel (AUX4, channel 6) before the M2 rig hand-over tests.
8. **Abort before the hand-over.** Your MPC ignores `/fleet/abort`; behind our mux, your next command re-arms the drone
   after our disarm in sim. BUILT (2026-09-29, Wesley's rulings): on /fleet/abort our mux latches and forwards only
   disarmed idle commands to the radio, whichever stream is selected, until it restarts. Our ESTOP, our flight faults
   and the operator DISARM (the rig's disarm button) fire it, so they ground your drones too; your own stream keeps
   publishing armed (the mux drops it). A fault of our tracker on a drone still on your stream does not fire it.
9. **One mocap owner.** Only one process can bind UDP 1511; your router has no SO_REUSEADDR. For combined rig runs, one
   node (your router, extended) should publish both stacks' topics.
10. **IRL config vs tests.** 7 of your IRL contract tests fail against the current configs (e.g. quad_rigid_body_id 12 in
    the config vs 7 in test_irl_m1_virtual_ring_contract.py).
11. **M2 join stall (sim, intermittent; ROOT CAUSE FOUND 2026-09-29).** 4 of our last 11 M2 runs stalled in drone_0's
    `M2_CPP_TRANSIT_CAPTURE`. Cause: when the C++ backend goes stale mid-transit for longer than the cached 2 s window,
    `c1f2_handoff.advance_reference_window` pads past the window with its terminal state; a window cut mid-transit is
    still moving, so the reference freezes the position but keeps ~0.22 m/s (the docstring assumes the window ends in a
    stopped hold). The C1F.2b recovery check then compares every fresh C++ reference (starting from the held drone at
    <= ~0.09 m/s) with that phantom velocity and rejects it forever (0.10 m/s tolerance); the backend, which thinks it
    has authority, trips its 0.05 m tube, brakes, replans, repeats. In the good runs the backend came back within
    0.25 s, inside the window. Our fork pads with a stopped hold (zero v and a past the window end); test
    `test_c1f2_padding_deadlock.py`. Still unknown: why the backend's cooperative-scene freshness check disables it
    mid-transit (`disableBackend` logs no reason) and why its replans then fail for ~3 s sim.
