# Note to Tejen before the combined M1 (draft 11 Oct, for Wesley to send)

Wesley: send it as it stands or cut it. Run ids are in brackets for us; strip them if you like. If Tejen still has the
9 Oct list (docs/tejen_questions_2026-10.md), this note replaces its items 4 (magnet), 6 (land_now) and 8 (who reviews
our edits). The others stand.

---

Hi Tejen,

We'd like to fly the combined M1 on **Wednesday 14 Oct in the afternoon**, after the morning measurements. Your
controller takes QUAD4 from its mat through the pickup, the drop into the net and the attach onto plate 9 of our
hovering ring. The plan:
- **R11a first:** the whole mission with your attach target 5 cm above the plate, so it cannot catch. We check
  everything around the attach, including our take-back.
- **R11b next:** the real contact attach, at your rig aim 1 cm below the face.

Can you be there, and are you OK with it?

**Your logs.** Could you send the logs or bags of a couple of your recent full tests (takeoff, pickup, drop in the
basket, attach)?
- We'd read your height error at contact and your kT in flight. They set R11a's offset and confirm the kT 22 we
  will type for QUAD4, which is built like QUAD2.
- Which version of your code was it, and was the payload on the ground or carried?

Below is what we changed around your stack, and what else we need from you. Every edit to our fork of your code is
logged in `src/tejen_mission/CHANGES_FROM_TEJEN.md` (branch m1-rig).

**1. Your kT estimator's height gate.**
- Your MPC learns kT only 0.20 m above the height it armed at, and it re-checks that on every tick.
- In our full-M1 twin, QUAD4 is released in the air at about 1.85 m, so the estimator never started. kT stayed at 22,
  your MPC flew about 11 cm low, and the tip hit the floor at the ball [R1503, R1505].
- In the twin we set the gate to -10 m when QUAD4 starts in the air. On the rig we keep your value.
- Note that on the rig your gate also pauses kT learning whenever QUAD4 flies below its arm height.

**2. The coil.**
- QUAD4 has one coil, and one node of ours (magnet_gate_3) is its only writer.
- Your commands pass until our attach span starts. From then on only ours pass, for good.
- Between your pickup and your drop, your OFF passes only in your pickup phases (your retry releases there) and in
  DROP_OBJECT / CLEAR_DROP_ZONE. An OFF anywhere else in the transit or over the ring is held, so a LANDING or a stray
  OFF cannot drop the puck onto the carriers. After the drop your OFFs pass again.
- Is that rule OK with you?

**3. Your supervisor.**
- In our combined launch its stop now gives up (our fleet takes QUAD4 back) instead of landing. Its default stays your
  disarm.
- Two reasons: your LANDING sends magnet OFF, which would drop the puck over the ring; and a stale input (the puck
  occluded) must never cut a flying vehicle.
- On the rig we do not run your supervisor for now; our launch refuses it.

**4. Your rig values are QUAD2's.**
- `irl_commissioning.yaml` has body 12, marker 22, marker-to-face -0.0225 m, magnet_drop_below_quad 0.575 m and kT 22.
  QUAD4 is body 14, marker 24, and is built like QUAD2, so we use your values. The puck is 60 g.
- Our rig launch refuses to start until the drop and kT are typed for QUAD4.
- We measure QUAD4's marker-to-face, taut length and contact height on Wed 14.
- Can you give us QUAD4's kT with and without the puck from R10a?

**5. A tilt cut over QUAD4.**
- We built a switch that, while your stream flies QUAD4, disarms it alone if it tilts past 60 deg for 25 ms, turns its
  coil OFF and lands the ring.
- It stays OFF on 21-22 Oct. It goes on from 28-29 Oct only if the props-off checks pass and you agree.
- We also propose a self-land in your planner, behind a switch that is off by default (taken_back_self_land): if our mux
  has not taken QUAD4 1 s after TAKEN_BACK, you fly out to 1.2 m from the ring, land and disarm.
- Would you add the same tilt rule inside your MPC, next to your stale-pose disarm?
- Do you have QUAD4 logs at transit height (2.3-2.9 m) that we could replay the rule on?

**6. Your transit after a mocap glitch.**
- In the twin we injected a false attitude on QUAD4 (65 deg for 0.1 s), twice [R1586, R1599].
- Both times your planner stayed in TRANSIT_TO_DROP_POINT until our timeout, about 150 s later.
- What does your planner do after a mocap glitch, and does it have a timeout of its own?

**7. Your object magnet when we take QUAD4 back before your pickup.**
- A take-back before your pickup (for example our LAND during DESCEND_TO_PICKUP) sends no magnet command, so the coil
  stays ON.
- In the twin QUAD4 then landed on the ball and caught it [R1624].
- Should TAKEN_BACK before the pickup send OFF?

**8. Your 0.30 s reference lead in ATTACH_READY** (attach_ready_target_lead_time).
- What did you measure to set it?
- Our contact attach flies 0. On the moving ring in the twin we flew both 0.30 and 0.

**What we need from you:**
- Your written OK on our fork of your code (CHANGES_FROM_TEJEN.md, branch m1-rig). A reply to this message is enough.
- Your OK on the coil rule (item 2).
- Your OK before any switch over your drone goes on (the tilt cut, the self-land). We turn none on without it.
- Logging during R10a on Thu 15: please keep your nodes' logs or a bag, so we can set QUAD4's values from them.

Thanks,
Wesley
