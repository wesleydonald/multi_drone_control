# Card: catch a release from the magnet marker (21 Oct; chain step 1b)

Wesley (21 Oct): a release "should detect it based on either the swung electromagnet being in the wrong place, or
the quad being too far away". The planner had only the second check (the cable stretched 6 cm past its length, 2
planner ticks). That check cannot see a quad that carries nothing, such as the T's quad opposite the gap: its release
moves nothing. Every carrier has a magnet marker on the rig (Motive bodies 21, 22, 23, 24 for QUAD1-4). **Bars
FROZEN 21 Oct 20:00**, before any run with the check.

## 1. Question
Does a magnet-marker check catch an unannounced release within about 0.1 s, including the unloaded quad's, without
firing in a normal flight?

## 2. Hypothesis
- **Detection.** A released magnet swings off its plate at once: on 0.55 m cables it hangs ~0.28 m out and ~7 cm
  lower within ~0.4 s, so it passes 4 cm beyond its attached offset in well under 0.1 s. Three samples in a row at
  60 Hz is 50 ms.
  - In the T, `t3_pair_release_tip` is detected within 0.5 s and the fleet resizes to the pair. Without the marker it
    is likely never detected (`t3_pair_release`, R2250+).
  - In the square, the marker catches the release ~0.15 s earlier than the stretch did (R2249: +0.21 s).
- **No false detection.** The attached marker sits at a fixed offset from its plate point, learnt while in band, and
  mocap noise is mm. So a whole floor-start flight never fires.

## 3. The one variable
`tip_detect_m` 0.04 (off by default):
- the planner subscribes `/drone_<d>/magnet_tip_state` for each carrier;
- it measures the distance from the marker to the plate point (the slot's attach point with the ring's pose
  applied), against the marker's attached offset, learnt in band (EMA 0.02 per sample);
- 3 samples in a row past it by 4 cm are a release. The action is the same as the stretch check's (drop or resize,
  per the drop rule).
- **Guards:** a stale ring pose (> 0.1 s); every fresh marker past at once (the ring's pose jumped); LAND (unless
  the drop rule needs it); the creep and lift phases and an unload, as the stretch check.

The stretch check stays on beside it.
- **Twin:** the chain's worlds publish each rod's pose (`--tether-pose-hz` 60), and the mocap emulator reports the
  rod's payload end on `/drone_<i>/magnet_tip_state` (`sim_io` `tether_tips:=true tether_len:=0.55`).
- **Rig:** `real_io_launch.py mocap_extra_bodies:=22:/drone_0/magnet_tip_state,23:/drone_1/magnet_tip_state,24:/drone_2/magnet_tip_state`,
  in drone order. No rig code changes.

Configs: `t3_pair_release_tip`, `t3_hover_tip`, `c4_release9_tip`. Each is its parent config plus `tip_detect_m` and
the emulated markers.

## 4. Baseline
- `t3_pair_release` (R2250/R2251, tonight): the stretch check only.
- `c4_release9_base` (R2249): stretch detection at +0.21 s; ring peak 57.1 deg at +0.6 s.

## 5. Pass / fail numbers (frozen 20:00)

| Run | Confirms | Falsifies |
|---|---|---|
| Unit tests | noise, a blip, a slow drift, a ring-pose jump, a stale ring pose, off, detached and LAND never fire; a 5.5 cm swing fires on the 3rd sample | any of these fails |
| `t3_hover_tip` x1 | no UNANNOUNCED DETACH from TAKEOFF to LANDED | any detection |
| `t3_pair_release_tip` x2 | "UNANNOUNCED DETACH ... magnet" within 0.5 s of the RELEASE, the resize to n=2, no drop or abort, tilt peak < 15 deg over the whole flight, the pair's roll < 5 deg p-p and lean < 5 deg over its last 10 s | no detection, a drop or abort, or a bar missed in either run |
| `c4_release9_tip` x1 | informational: the detection time against R2249's +0.21 s, and the roll peak | - |

**Rig go/no-go for Part C:**
- the unit tests and `t3_hover_tip` pass;
- `t3_pair_release_tip` passes;
- tomorrow's bench reads every marker in Motive (bodies 22, 23, 24) and `/drone_<i>/magnet_tip_state` streams while
  props are off;
- plus the drop-rule card's own conditions.

## 6. Repeats
`t3_pair_release_tip` x2; the others x1.

## 7. Cost
4 laptop runs (~5 min each), recorded.

## 8. New code?
Yes:
- `dissipative_node`: `_tip_check`, `_tip_cb`, and `_on_unannounced` (shared with the stretch check, behaviour
  unchanged);
- the `tip_detect_m` knob (`common.yaml`, `real_mode.PLANNER_OPTIONAL`);
- `generate_rigid_world --tether-pose-hz` (the chain's worlds regenerated);
- `payload_mocap_emulator` `tether_pose_topic` / `tether_len` and `rod_end`;
- `sim_io_launch` `tether_tips` / `tether_len`;
- tests: `test_tip_detect.py`, `test_magnet_tip_emulation.py`.

If it fails, the knob stays off and the code is removed.

## Critic (21 Oct ~20:15): REWRITE. Summary of the must-fix items; the full report is in the session

1. **No rig evidence yet.** Bodies 21-23 have never been seen streaming, and `/drone_<i>/magnet_tip_state` is not
   bagged (`tools/rig_flight.sh:81-90`).
   - Record the markers from Part A on, with the detector OFF.
   - Replay the check over those bags and gate on zero would-fires through lifts, paths and LANDs.
   - Bench: move each magnet by hand, cover each marker for 2 s, record each offset over 3 re-seats.
2. **Invalid samples.**
   - A NaN or a (0,0,0) untracked body must be rejected.
   - Reject any sample whose distance to its own drone is outside the taut length +-3 cm (the guard's band,
     `GUARD_TIP_BODY_M`).
   - Catch rotation glitches as well as translations.
3. **Use the ring-frame vector, not the scalar distance**, as `_release_seen_tick` does. On the rig the marker sits
   ~3 cm above the plate point, so a tangential swing needs ~8 cm before the norm grows 4 cm (~0.17 s).
4. **Learn the offset once**, in a still window after the lift.
   - Bound it to +-1 cm.
   - Write N and alpha in seconds; the rig streams ~100-120 Hz against the twin's 60.
   - Refuse to arm a drone whose offset is over ~6 cm.
   - The unbounded EMA follows a slow peel-off, so that is never a release.
5. **Wire the rig path:** `tip_detect_m` in the params file, `mocap_extra_bodies` on T1, and each drone's offset
   logged after the lift.
6. **Rig prediction.**
   - The plate-3 quad is planned at 0.1 N against a ~0.7 N tether, so OFF may leave its magnet resting on the plate.
     Only the twin's rod, which has no collision, falls away.
   - So "no detection, marker still on the plate" is expected, not a failure. "Marker off its plate in the bag, no
     detection" falsifies.
   - Sheet: with no detection, `/fleet/detach 0` (the announced path, which leaves only an opposite pair) before LAND.

Should-fix 7-15:
- a 0.2 s bar;
- twin marker noise and dropouts;
- the baseline with `tether_tips` on and the detector off;
- contiguous samples;
- the lift guard (the card text is wrong: the lift is not guarded);
- tests for LAND with the ring up, a NaN, and plate 0 in the T;
- planner load (`tools/load_check.py`);
- the regenerated world SDFs need Wesley's word, and the thresholds need re-sizing for 1 m;
- the one-tree rule for the params file.

**Decision for tonight:** the detector stays on branch `tip-detect`, not merged, and is not flown. Tomorrow's rig
only records the markers (detector off) for the evidence in item 1.

## Rev 2 (21 Oct ~23:15), after the critic and Wesley

**Wesley (23:00) on the critic's main point:** a magnet switched off on the rig "will instantly be noticeable and
absolutely detach because of the angle the quad would be at". So on the rig the released magnet is expected to leave
its plate at once, as in the twin.

| Must-fix | Rev 2 |
|---|---|
| 1. Rig evidence | The sheet routes markers 22-24 on every flight with the check OFF, and `rig_flight.sh` bags `/drone_<i>/magnet_tip_state` (main tree cf556ff). Bench: each magnet moved by hand, each marker covered, 3 re-seats. Part C uses the check only if this card passes tonight; the B flights' bags are the first false-fire evidence |
| 2. Invalid samples | Non-finite samples are dropped. A sample whose body-to-marker length is off the learnt one by more than 3 cm (`TIP_BAND_M`) is not this magnet: an untracked body at 0,0,0, or a swap. A ring-pose jump moves every armed marker at once and is not a release (tested by moving the ring's pose) |
| 3. Scalar distance | Now the ring-frame vector from the plate point (`R^T (tip - ring) - rho`), against the learnt vector |
| 4. The learnt offset | Learnt once: the median over `TIP_LEARN_S` 1.0 s of still ring (< 0.05 m/s) after the lift, then fixed. A drone is not armed, with a loud WARN, if its offset is over 6 cm from its plate or its body-to-marker length is over 6 cm from the rod. Counts are in seconds: `TIP_DETECT_S` 0.03 s contiguous, a gap over `TIP_GAP_S` 0.05 s restarts it |
| 5. Rig wiring | `configs/rig/chain_pair_tip.yaml` (`tip_detect_m` 0.04). T1 routes the markers. Each drone logs one "armed: offset ..." line after the lift |
| 6. Prediction and the operator's branch | Prediction (Wesley): the marker leaves at once, so detection within ~0.1 s. Rig falsifier: a would-fire in A/B (from their bags) or no detection within 0.2 s of the OFF. The sheet keeps the `/fleet/detach 0` branch, Wesley's call |
| 11. LAND and the lift | The check is OFF in LAND (the stretch check still runs) and cannot arm before the lift is done |
| 8. Twin realism | 0.5 mm marker noise in the emulator |
| 9. Control | `t3_pair_release_tipoff`: the markers emulated, the check off |

**Bars (rev 2, frozen 23:15):**
- `t3_hover_tip` x1: every drone armed, no detection.
- `t3_pair_release_tip` x2: "UNANNOUNCED DETACH ... magnet" within 0.2 s of the RELEASE, and the resize to n=2.
- `t3_pair_release_tipoff` x1: no magnet detection, by construction.
- `c4_release9_tip` x1: informational, the detection time against R2249's +0.21 s.
- Unit tests: 21 pass (`test_tip_detect.py` rev 2, `test_drop_and_land.py`); the tracker log test passes.

**Also on this branch:** the tracker logs the raw measured cable accel (`acm_x/y/z`) beside the applied one, so peak
pulls above the 6.6 N cap show (Wesley 21 Oct).

## Critic round 2 (21 Oct ~23:00): NO-GO for the check ON on Thu 22; rewrite (rev 3) before it flies

Must-fix for rev 3:
- **M1. A ring-pose tilt glitch fires plate 0 or 6, then DROP AND LAND from 1 m.** In the T, a ~10 deg Motive
  mis-solve about the 3-9 axis for >= 30 ms moves markers 0/6 ~4.8 cm in the ring frame and marker 3 ~1 cm, so the
  "every marker at once" guard misses it. Fix: require the marker to move in the world frame (a pose glitch moves the
  plate point, not the magnet) and/or a ring-quaternion step guard with a hold-off (as the trackers' drop_glitch_step_deg
  15). Add a test on plate 0 that does not stub `_on_unannounced`.
- **M2. Persistence too short.** Time on header stamps, require a minimum sample count, and use 0.06-0.1 s (the rig's
  QUAD4 guard uses 0.1 s).
- **M3. A lone armed marker has no glitch guard.** Act only with at least one other armed, fresh marker; otherwise log
  WOULD-FIRE.
- **M4. Frozen markers.** Reject bit-identical repeats. The bench's covered-marker result is a gate.
- **M5. The rig replay.** Now `tools/tip_replay.py` (7102723). Run it over Thu 22's A/B/C bags (markers recorded with
  the check OFF) and today's QUAD1 marker-21 data.
- **M6. Restore the whole-flight bars:** no drop or abort, no detection on drones 0/2, and the pair's roll and tilt no
  worse than R2250/R2251.

Should-fix:
- S1: the still window, from a window median or position spread, logging "still learning" after ~5 s;
- S2: clear the tip state on attach and detach;
- S3: a load check;
- S4: the twin tip configs fly the rig's params file;
- S5: keep the rod-pose publishers in separate `*_tip` worlds, or get Wesley's word;
- S6: a tighter ring-pose staleness;
- S7: stale comments;
- S8: score in sim time.

The tracker's raw-pull logging (acm_x/y/z): **GO, low risk.** It is ported alone to day-1021.

**Decision (Wesley's condition "if it passes its critic and twin checks"):** Part C flies Thu 22 with the check OFF
(chain_pair_noxy.yaml, the stretch check), and LAND if nothing detects within 2 s. The markers are bagged, so Part C's
bag gives the check its first rig evidence: the detection time it would have had, and false fires replayed
offline.
