# Note to Tejen before Wednesday (final 12 Oct, for Wesley to send)

Wesley: send it as it stands or cut it. This replaces items 4, 6 and 8 of the 9 Oct list
(docs/tejen_questions_2026-10.md); the rest of that list stands.

---

Hi Tejen,

**Wednesday 14 Oct, afternoon.** We'd like to fly the combined flights with your controller, after our morning checks
and our own flights. In order, each one only if the one before went well:
- **R6:** your whole mission (pickup, drop into the net, attach) with your attach target 5 cm above plate 9, so it
  can't catch. We check everything around the attach, including our take-back.
- **R7, R8:** the real contact attach at your rig aim, twice.
- **R9, if R7 and R8 are clean:** the full M1. QUAD4 starts attached to the ring. We lift, detach QUAD4 in the air
  and hand it to your stream. You pick up the puck, drop it in the net and reattach.

Thursday 15:00-18:00 is for retries of anything that needs a fix.

Can you be there, and are you OK with this plan?

**Questions:**
1. **R9 starts your MPC in the air** (~1.85 m). Your kT estimator learns only 0.2 m above the height you armed at. If
   you ARM on the floor before we take off, it should learn as normal. Is that right? Is anything else in your stack
   unhappy with a start in the air?
2. **Your logs.** Could you send logs or bags of a couple of recent full tests (takeoff, pickup, drop, attach)? Please
   keep your nodes' logs or a bag on Wednesday too. We use them to check QUAD4's kT (we type 22, as for QUAD2, since
   QUAD4 is built the same) and your height error at contact.
3. **Your 0.30 s reference lead in ATTACH_READY:** what did you measure to set it? Our contact attach flies 0.
4. **A take-back before your pickup** (for example our LAND during DESCEND_TO_PICKUP) sends no magnet command, so
   the coil stays ON (in the twin QUAD4 then landed on the puck and caught it). Should TAKEN_BACK before the pickup
   send OFF?
5. **After a mocap glitch in transit** (a false 65 deg for 0.1 s), your planner stayed in TRANSIT_TO_DROP_POINT until
   our timeout, twice in the twin. Does it have a timeout of its own?

**What we need from you:**
- **Your OK on our fork of your code.** Every change is listed in `src/tejen_mission/CHANGES_FROM_TEJEN.md`. A reply
  to this message is enough. The main ones:
  - your supervisor's stop gives up (our fleet takes QUAD4 back) instead of landing;
  - your QUAD2 values on QUAD4 must be typed (body 14, marker 24);
  - your kT gate stays your value on the rig.
- **Your OK on the coil rule.** QUAD4 has one coil, and one node of ours is its only writer. Your commands pass until
  our attach starts, then only ours. Between your pickup and your drop, your OFF passes only in your pickup and drop
  phases. A stray OFF over the ring can't drop the puck onto the carriers.
- **Your OK before any switch over your drone goes on.** Two are built and both stay off on Wednesday:
  - a tilt cut that disarms QUAD4 alone if it tips past 60 deg while you fly it;
  - a self-land after our take-back.

  Would you add the same tilt rule inside your MPC?
- **Thesis credit.** I'd describe your part as: "Tejen's controller, with the changes listed in Appendix X made by
  the author with his agreement", and cite your thesis. Is that OK, or would you like it worded differently?

Thanks,
Wesley
