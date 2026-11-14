# Reply to Te-Jen's review of the 4 Nov package (Wesley sends)

Thanks, this was very useful. Point by point:

1. **The drop gate.** Agreed, and our numbers match yours.

2. **Trim.** Agreed. We won't touch your drop trim, and it stays quiet-only.

3. **The reference in k1m_3.** You're right. Our run.csv `d3_ref_z` was our idle tracker's reference, which follows
   the measured drone while your stream flies QUAD4. The flight table now also carries, from the bag, your
   `/join_planner/reference`, your throttle, the throttle sent to the radio, and who owns QUAD4.
   **The throttle limit is confirmed.** Your OCP bounds the throttle state at 0.60 (`acados.py`, `ubx` on state 15).
   Our `throttle_max:=0.95` only reaches our trackers, never your controller.
   - With the object on board, your throttle had a median of 0.57-0.60 in the transit. It sat at the cap for 17.2 s in
     k1m_3, 4.1 s in k1m_4, 1.5 s in k1m_2, and 9.2 s of k1m's lift.
   - Without the object it was 0.44-0.49 and never reached the cap. So nr9m_2's 25 cm stall (throttle 0.48) was not
     the cap.
   - Our measured thrust map puts QUAD4's hover throttle with the object at about 0.60 on a ~21.5 V pack.

   In our copy, `mpc_throttle_max` (0.60-0.95, your 0.60 unless typed) now sets that bound at run time, with no
   rebuild. **Is there a reason the 0.60 should stay (your thrust model's range, motor limits, safety)? What value
   would you fly?**

4. **Forecast churn.** We'll look at the ring-velocity input (our mocap twist, slope-filtered) and its extrapolation
   next.

5. **Higher drops.** Agreed on all of it:
   - We will set the release height above the net's surface, measured, and reconcile it with the 1 m cable and the
     2.6 m ceiling.
   - Our `drop_on_arrival` (off by default) keeps your height gate. You're right that it drops the speed condition.
     We'll change it to release when the predicted landing point is inside the basket (the magnet's position plus its
     velocity times the fall time), rather than when the magnet passes through the radius.
   - Also new in our copy, off by default: `lift_full`. Your loaded lift ends at its full height before the transit,
     so QUAD4 goes up and then over. That's what my supervisor asked for, after the magnet swung into the carriers'
     cables.

Order from here: the throttle limit (done, flying in simulation now), then the churn and the drop entry, then the plate
1/plate 9 discrepancy and the UKF rejection.
