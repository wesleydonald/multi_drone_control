# For Te-Jen, from the lab day of Thu 5 Nov (the message Wesley sends; evidence in registry RIG-1105-*)

Hi Te-Jen, a quick update from today on the 0.86 kg ring with 1 m cables.

**What worked**
1. The first full detach and re-attach on the rig: four quads on a square, QUAD4 released from its plate, flew out, came
   back and caught on the first try (17 mm off the plate centre), proven and folded back in 14.4 s after the release.
2. Your pickup and the drop into the flying ring: QUAD4 picked up the small object first try, climbed straight up and
   dropped it into the net on arrival. With the ring moving on a 0.3 m circle it also dropped the bigger object into
   the net, 4 cm from the centre.

**What we typed in our copy** (no code change during the day; one after it, question 2)
1. `contact_dz:=-0.03`: your descent aims the magnet 3 cm below the plate instead of 1 cm. At -0.01 the magnet stopped
   1.7-2.3 cm above the plate on every try; at -0.03 it touched on the first try.
2. The bigger object: contact height 0.0181 m (measured with the magnet face on it), 100 x 10 mm.

**Questions**
1. On the triangle (carriers 60 deg either side of the plate) your descents reached plate height 3.5-10 cm off the plate
   centre, with QUAD4 2-7 cm off your reference sideways and the magnet swinging 2-6 cm under it, so no catch in about a
   minute. The square caught first try. Any setting you would change for the final alignment? We kept `match_xy` at 0.08
   because we worry a tighter value stops the attempts altogether.
   Measured afterwards (bags of nr9m, nr9o_2, nr1r_3: 16 descents to plate height on the triangle): the misses are mostly
   radial, alternating inward and outward (-7.8 to +9.5 cm), sideways mostly within 4 cm (two at 9-10 cm). Both parts
   add up: QUAD4's body was -4 to +11 cm off radially at the bottom and had moved 3-10 cm radially in the last second,
   and the magnet swung -10 to +7 cm radially under it. The square's catch (sq8_2, 1.7 cm) came with the body 7 cm
   inward and the swing 6 cm outward. Does your final alignment chase the measured tip (`measured_tip_trim`) while the
   magnet swings on its 1 m cable?
2. When your TAKEOFF arrives before our release has finished (our route round the outside took ~20 s), your reference
   starts from where QUAD4 was at the TAKEOFF and stays there; at the hand-over QUAD4 flew 1.3 m across the ring. In our
   copy of your planner (after the day; CHANGES_FROM_TEJEN.md), a TAKEOFF before our release is now held and carried out
   at the release from where QUAD4 is; while held your reference is QUAD4's own pose, since your MPC gets the same
   TAKEOFF and flies (our mux drops its stream until the release). Does your planner or MPC expect the TAKEOFF only from
   a quad at rest?
3. Your whole-body safety planner runs as diagnostics only on the rig (`vectorised_whole_body_diagnostics_only`). Is it
   ready to enforce, in your view?
4. After a re-enabled pickup body in Motive with its centre a few cm off, your pickup reported "object attached but the
   tip 6-7.5 cm off target" and retried three times. That was our setup error, but is that check's 5 cm threshold typable?

**Changed in our copy of your code after the day** (all in CHANGES_FROM_TEJEN.md)
1. A TAKEOFF pressed before our release is held and carried out at the release, from where QUAD4 is then (question 2);
   while held, your reference is QUAD4's own pose. A TAKEOFF your planner refuses while QUAD4 is in the air (e.g. no fresh
   ring pose) no longer sends DISARM: it holds where QUAD4 is and tries again at the next release message.
2. Your C++ backend's carrier-tracking check (the one that disables the backend when a carrier is off its committed path)
   now allows 0.20 m instead of 0.05 m (new parameters `cooperative_tracking_check_half_{x,y,z}_m` in
   c1f6_moving_rendezvous.yaml); the collision inflation stays 0.05 m. Why: in our simulation the three-carrier ring
   hovers 5 cm off its planned point (we fly it without the sideways integral) and the 0.05 m check latched your backend
   off for good ~2 s into the transfer, so QUAD4 never reached the plate. At 0.20 m the low route still reaches its end
   point (at 0.20 m inflation it would not).
3. In simulation one pickup cell flies your over route: on the low route your C++ planner stops 0.7 m short of the drop
   point with the object (the plate-3 carrier's tether against QUAD4's cable in 105 of 127 failed checks).

**One more finding from the day:** the ring drop after the bigger object's drop (k1ob_2) was the object's impact in the
net (it hit 14 cm off centre at ~2.8 m/s) pulling two carriers' magnets off; not your planner. We will only drop the small
object from now on.
