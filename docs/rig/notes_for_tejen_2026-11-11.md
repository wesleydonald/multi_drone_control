# For Te-Jen, from the lab day of Wed 11 Nov (the message Wesley sends; evidence in registry RIG-1111-*)

Hi Te-Jen, an update from Wednesday on the 0.86 kg ring with 1 m cables (carriers on the triangle, QUAD4 on plate 9).
The logs and bags are in the two zips.

**What worked**
1. Your re-attach on the triangle caught on the first descent in 3 of 3 flights (nrtfc_2, nrtfc_3, nrtfc_5). Two things
   changed since 5 Nov, when none of 37 descents caught there: a funnel on the plate, and your bands at 5 cm
   (`match_xy:=0.05`, `descent_abort_xy:=0.05`, dwell 0.5 s; they were 0.08 and 0.10). Contact 4.9, 13.8 and 7.9 mm from
   the plate centre, 17-22 s from the release to the fold-in. The tighter band did not stop the attempts: the descent
   paused 3-4 times per flight with the tip 5.0-5.3 cm off and went on. (In nrtfc_3 the magnet let go 4 s after the
   fold-in: a loose lead on our magnet, fixed.)
2. Your drop into the circling ring: the object landed in the net 3-8 cm from the centre (k1zo) and 3 cm (fm9hc_8).

**What we typed in our copy**
1. The release height: `drop_release_clearance` 0.03 (yours 0.14). At 0.14 the object was let go about 0.23 m over
   the ring and its impact pulled a carrier's magnet off the ring (k1zo: the ring went from 5 to
   76 deg in 0.4 s). At 0.03 the drop was clean (fm9hc_8).
2. The object now has a funnel on it. Its contact point is 11 mm over the Motive pivot and 13 mm to the side, typed as
   `payload_pickup_point_x/_y/_z` and the manager's `pickup_point_offset_x/_y/_z`.

**Questions**
1. The pickup is what stops the full mission. 4 of 11 descents lifted the object: the 4 that reached
   MAGNET_ATTACH_WAIT within about 3 cm of its centre lifted within 1.5 s, and the 7 that arrived 4.7-9.7 cm off did
   not ("fresh measured vertical separation; retry"), with the ring circling or at rest. In those descents the
   magnet swung 4-9 cm (standard deviation) under QUAD4 on its 1 m cable, and your reference was 1-5.5 cm from the
   object. In `measured_tip_tiered` the LOW band's 10 cm only gates the crossing into the last 8 cm. In our copy the
   height can now also freeze inside the LOW band while the tip is more than 4 cm off
   (`pickup_alignment_low_hold_xy_m`, off unless typed). Not flown yet. Is there a setting you would use instead, and
   did your one-quad pickups on the 1 m cable swing this much?
2. After a failed lift your retry goes to the object wherever it has come to rest. In fm9c_6 the bigger object came off
   the magnet 7.5 s into the lift while swinging, fell from 0.5 m and slid to (-0.07, 2.16) m, 0.4 m further out than
   where it started. Your reference went from (-0.35, 1.75, 1.98) to (0.25, 2.15, 2.54) m in 2.4 s (up to 0.8 m/s
   sideways) and QUAD4 overshot it to (0.29, 2.34, 2.89), where the cameras lost it. Is there a bound on where the
   pickup may go? We could not find one, so our copy now has a typed box (`pickup_box_m`, 1.9 m on our rig): an object
   lying outside it is a give-up and our side lands QUAD4.
   The overshoot of 0.35 m upward came right after the object (about 0.15 kg) came off: does your mass estimate take a moment to drop it?
3. When Motive loses a body, its last pose keeps arriving at the full rate, so a timeout on arrival times does not see
   the loss. In fm9c_6 QUAD4's pose stood at (0.29, 2.34, 2.89) for the 4.9 s it flew on, 0.8 s of them on your
   controller and the rest on ours. The object's pose does the same under QUAD4 (frozen for the whole descent in fm9c_6). Does
   your MPC's 0.25 s pose timeout read arrival times only?
4. With the bigger object on the circle your `TRANSIT_TO_DROP_POINT` never passed its arrival gate (0.18 m, 0.20 m/s for
   0.5 s) in 29.5 s (fm9c_4). Is there a setting for a moving drop point, or is the gate meant for hover?

**Changed in our copy of your code since 5 Nov** (all in CHANGES_FROM_TEJEN.md)
1. Your drop check waited 0.4 s and gave up while the object was still on the magnet; it left 0.8-0.9 s after the OFF
   (fm9hc_8). It now waits up to 2 s (`drop_puck_wait_s`) before giving up.
2. A FORCE DROP button (`/join_planner/force_drop`): in TRANSIT_TO_DROP_POINT or SETTLE_ABOVE_DROP_POINT it goes
   straight to the lowering, in the lowering it releases; only with the magnet within 0.25 m of the drop point. The
   mission then carries on.
3. The typed arguments above (`drop_release`, the LOW hold, the pickup point's sideways offset).
4. One bug was ours, not yours: our magnet gate held your OFF when it arrived 1 ms before your DROP_OBJECT phase
   (k1zo_3). Fixed.

**Thursday 12 Nov**: hover re-attach repeats first, then the full mission in hover, then on the circle.
