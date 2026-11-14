# For Te-Jen, from the lab day of Wed 4 Nov (the message Wesley sends; evidence in registry RIG-1104-*)

Hi Te-Jen, here is what happened with your planner and controller on the rig today, on the 0.86 kg ring with 1 m cables.

**What worked**
1. The first real join: your controller flew QUAD4 down to plate 9, the magnet caught and our check confirmed it on the
   second contact; then we folded it in as a fourth carrier (sq6m). Your descent hold from your review did its job: it
   held at 10 cm and resumed instead of restarting the approach.
2. The pickup: QUAD4 picked the object up and carried it over the ring on three flights.
3. The release: on six flights our planner unloaded QUAD4, stepped it out and handed it to your mission, every time.

**What we changed in our copy** (all logged in CHANGES_FROM_TEJEN.md, with tests)
1. Your TAKEOFF after an air release started from the mat it booted on, so QUAD4 dropped from 2.24 m to ~1.1 m and swung
   back across the ring. Now a TAKEOFF more than 0.5 m above the boot ground starts where QUAD4 is.
2. Your pickup proof (15 mm drift, 0.03 m/s) never proved on the 1 m cable: the gap moved ~2 cm as the object left the
   floor. It is now typable; 50 mm / 0.08 m/s proved.
3. The drop never started. Over the ring the object swung 0.23-0.26 m (median) round the drop point on the cable, so
   your arrival check (0.18 m, held 0.5 s) passed at most once. We added a switch that drops at the transit height as
   soon as the magnet is within 0.12 m of the centre (simulation: in the net 3.5 cm from the centre).
4. In progress: the lift ends once the object is 8 cm off the floor and the transit then climbs diagonally toward the
   ring, which swung the magnet into the carriers' cables a few times. We are making it climb straight up to the
   transit height first, then go level. Also a voltage term for your thrust ratio (below).

**Questions**
1. Your MPC's thrust ratio is fixed at 22 with no voltage term, and the UKF waits for the takeoff height. After QUAD4
   had carried the ring its pack was low and it sat 25 cm under its reference for 38 s (nr9m_2). In that flight the UKF
   estimates were 31-43 and were rejected on NIS. Do you know why they were so far from 22?
2. In k1m_3 your transit reference sank from 2.40 to 2.01 m while still in TRANSIT_TO_DROP_POINT, which put the
   object down at the ring's height. Do you know what lowers it in that phase?
3. Your IRL attachment inference flipped between ATTACHED and LOST 3-5 times per pickup flight while the object was on
   the magnet. Is that expected on a swinging 1 m cable?
4. On plate 9 your descents ended 24-59 mm from the target (the plates are 27.5 mm in radius); on plate 1 they ended
   3-4 mm away. Any idea why one plate is so much worse?
5. My supervisor wants us to try a smaller object and to drop it 0.5-0.6 m above the ring. Anything in your planner
   that would stop a release from that height?

I can send the logs if you want to look (about 7 MB for the seven flights that matter).
