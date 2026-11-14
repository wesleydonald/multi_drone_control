# Real-world attach, re-attach and pickup descents, 4-5 Nov 2026 (exploratory)

Made by `summarise.py` from `joins.csv` and `flights.csv` (made by `joins.py` from the launch logs and bags in `results/rig/2026-11-04/` and `-05/`). These are exploratory readings of recorded flights: no bars, numbers as measured. `joins.csv` has one row per descent of QUAD4's magnet toward its target. A descent is each entry into his DESCEND_TO_ATTACHMENT or DESCEND_TO_PICKUP, including the re-descent after a failed proof. It also has one row per drop into the net.

**How the offsets are read.** For a plate, the reference is the weld point the launch typed for that plate: the magnet-tip marker's own position when welded on it, in the ring's frame. Height is measured along the ring's normal above that point, as our contact check measures it. Horizontal offsets are along the plate's radius projected on the floor (+ outward) and anticlockwise seen from above (+). The tilted ring frame is not used for them, so the body 1 m up is not thrown sideways when the ring leans. **At plate height** means the first tip sample at or below the weld point's height, or the lowest sample when the magnet settled within 1 cm above it. A descent that never got within 1 cm is read at its lowest tip sample ("never at plate height"). The second reading, **lowest**, is the tip's lowest sample in the descent: where a magnet that slid off the plate ended up. Body is QUAD4's mocap body at the same instant, swing is tip minus body, and "his reference" is /join_planner/reference. For a pickup, the reference is the object's Motive body in x and y (world frame), and the height is the gap above the typed pickup_contact_height. The miss is the tip's horizontal distance from the target centre at that reading. The M2A plates are 5.5 cm circles; our contact radius is 2.5 cm (for reference, not a bar).

**Check of the geometry.** At every CONTACT line in the logs, the tip read from the bag at the contact time reproduces the line's xy and normal: 14 lines, all within 0.4 mm in xy and 0.3 mm in height. The usable welded starts of 5 Nov sit 0.9-5.6 mm in xy and within 1.6 mm in height (8 flights) from the typed weld point (geometry table at the end).

## Overall, by kind x carriers x ring motion

| kind | carriers | ring | days | flights | descents | at plate / target height | contacts | caught + proven | miss at height, cm: median (range) | miss within 2.75 cm | lowest point, cm: median (range) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| attach from the mat | triangle | hover | 04 Nov | 2 | 3 | 2 | 2 | 1 | 2.4 (1.9-2.8) | 1 of 2 | 2.4 (1.9-2.8) |
| pickup | triangle | hover | 04 Nov, 05 Nov | 6 | 10 | 10 | 8 | 5 | 3.4 (1.7-12.9) | 3 of 10 | - |
| pickup | triangle | orbit | 05 Nov | 2 | 5 | 5 | 4 | 1 | 6.1 (2.0-10.2) | 1 of 5 | - |
| re-attach | square | hover | 05 Nov | 1 | 2 | 1 | 1 | 1 | 1.6 (1.6-1.6) | 1 of 1 | 1.6 (1.6-1.6) |
| re-attach | triangle | hover | 04 Nov, 05 Nov | 6 | 37 | 31 | 11 | 0 | 6.2 (0.3-11.0) | 3 of 31 | 7.5 (0.3-16.5) |
| re-attach | triangle | orbit | 05 Nov | 1 | 4 | 2 | 0 | 0 | 8.6 (8.3-8.9) | 0 of 2 | 7.8 (7.4-8.2) |
| **all plate descents** |  |  | 4-5 Nov | 11 | 46 | 36 | 14 | 2 | 6.2 (0.3-11.0) | 5 of 36 | 7.2 (0.3-16.5) |

"Contacts" counts every descent that ended in a declared contact or a pickup attach, proven or not. For a pickup it also counts the object rising with the magnet. "Caught + proven" means our co-motion proof passed (plate), or his pickup proof passed and the transit started (object). The 1.15 kg ring flew one descent (sq6b, the triangle, from the mat); every other flight was on the 0.86 kg M2A ring.

## How the descents ended

| kind | outcome | detail (his phase reason or our proof) | descents |
|---|---|---|---|
| attach from the mat | aborted | take-back (LAND / give-up) | 1 |
| attach from the mat | caught+proven | proven (co-motion) | 1 |
| attach from the mat | contact not proven | failed (inconclusive) | 1 |
| pickup | aborted | take-back (LAND / give-up) | 1 |
| pickup | caught+proven | his pickup proof passed, transit started | 6 |
| pickup | caught, not proven | the object rose with the magnet; his proof never passed | 1 |
| pickup | contact not proven | attached, released by his check: tip off target | 4 |
| pickup | contact not proven | attached, the object came off in his lift proof | 1 |
| pickup | miss | no attach (no progress in 10 s) | 2 |
| re-attach | aborted | alignment lost in the descent | 3 |
| re-attach | aborted | take-back (LAND / give-up) | 2 |
| re-attach | caught+proven | proven (co-motion) | 1 |
| re-attach | contact not proven | failed (inconclusive) | 3 |
| re-attach | contact not proven | failed (lost) | 7 |
| re-attach | contact not proven | failed (separated) | 1 |
| re-attach | miss | moving or far off | 4 |
| re-attach | miss | quiet miss (no progress) | 2 |
| re-attach | miss | sank off the plate edge | 20 |

## Per flight

| day | flight | ring | carriers | ring motion | kind | target | descents | at height | contacts | caught + proven | miss at height, cm: median (range) | release (or his TAKEOFF) to proven | note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 04 Nov | sq6b | 1.15 kg | triangle 3/11/7 | hover | attach from the mat | plate 9 | 1 | 0 | 0 | 0 | - | - |  |
| 04 Nov | sq6m | M2A | triangle 3/11/7 | hover | attach from the mat | plate 9 | 2 | 2 | 2 | 1 | 2.4 (1.9-2.8) | 24.8 s from his TAKEOFF |  |
| 04 Nov | nr9 | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | body 24 tracked QUAD2's magnet all flight; his approach never completed |
| 04 Nov | nr1_2 | M2A | triangle 3/11/7 | hover | re-attach | plate 1 | 3 | 3 | 3 | 0 | 0.4 (0.3-0.4) | - |  |
| 04 Nov | nr9m | M2A | triangle 3/11/7 | hover | re-attach | plate 9 | 4 | 3 | 0 | 0 | 6.6 (6.2-7.0) | - |  |
| 04 Nov | nr9m_2 | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | his TAKEOFF stuck 25 cm low (before the takeoff from the air) |
| 04 Nov | nr9m_3 | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | bad mocap start; LAND 8 s after the TAKEOFF |
| 04 Nov | nr9m_4 | M2A | triangle 3/11/7 | hover | re-attach | plate 9 | 5 | 3 | 3 | 0 | 8.0 (5.9-9.7) | - |  |
| 04 Nov | nr9m_5 | M2A | triangle 3/11/7 | hover | re-attach | plate 9 | 5 | 5 | 3 | 0 | 6.1 (5.4-7.6) | - |  |
| 04 Nov | k1m | M2A | triangle 3/11/7 | hover | pickup | object | 1 | 1 | 1 | 0 | 2.4 (2.4-2.4) | - |  |
| 04 Nov | k1m_2 | M2A | triangle 3/11/7 | hover | pickup | object | 2 | 2 | 1 | 1 | 7.9 (3.0-12.9) | 47.1 s from his TAKEOFF |  |
| 04 Nov | k1m_3 (+c) | M2A | triangle 3/11/7 | hover | pickup | object | 2 | 2 | 2 | 1 | 3.9 (3.4-4.4) | 46.7 s from his TAKEOFF |  |
| 04 Nov | k1m_4 | M2A | triangle 3/11/7 | hover | pickup | object | 1 | 1 | 1 | 1 | 3.3 (3.3-3.3) | 20.3 s from his TAKEOFF |  |
| 05 Nov | k1d | M2A | triangle 3/11/7 | hover | pickup | object | 1 | 1 | 1 | 1 | 1.7 (1.7-1.7) | 29.9 s from his TAKEOFF | drop: hit the net |
| 05 Nov | fm9_2 | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | the lift bounced to 30.6 deg (Motive recording): automatic drop |
| 05 Nov | fm9_4 | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | the lift bounced (Motive recording); LAND at +7.5 s |
| 05 Nov | fm9s | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | slack cables in the lift; automatic drop in the descent |
| 05 Nov | fm9y | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | VOID: the ring set down one plate rotated |
| 05 Nov | fm9y_2 | M2A | triangle 3/11/7 | hover | pickup, re-attach | object, plate 9 | 6 | 6 | 2 | 1 | 6.3 (1.9-10.7) | 104.4 s from his TAKEOFF | drop: hit the net |
| 05 Nov | nr1r | M2A | triangle 3/11/7 | hover | - | - | 0 | 0 | 0 | 0 | - | - | QUAD4 started off its plate; dashed across at the release; LAND |
| 05 Nov | nr1r_3 | M2A | triangle 3/11/7 | hover | re-attach | plate 1 | 8 | 5 | 2 | 0 | 8.2 (3.4-11.0) | - |  |
| 05 Nov | sq8 | M2A | square 11/2/5 | hover | - | - | 0 | 0 | 0 | 0 | - | - | no start weld (QUAD4 not on its plate); LAND |
| 05 Nov | sq8_2 | M2A | square 11/2/5 | hover | re-attach | plate 8 | 2 | 1 | 1 | 1 | 1.6 (1.6-1.6) | 14.3 s from release |  |
| 05 Nov | nr9o_2 | M2A | triangle 3/11/7 | orbit | re-attach | plate 9 | 4 | 2 | 0 | 0 | 8.6 (8.3-8.9) | - |  |
| 05 Nov | k1o | M2A | triangle 3/11/7 | orbit | - | - | 0 | 0 | 0 | 0 | - | - | no pickup-object pose (body 6 not streamed); LAND |
| 05 Nov | k1o_2 | M2A | triangle 3/11/7 | orbit | pickup | object | 4 | 4 | 3 | 0 | 5.8 (2.0-10.2) | - |  |
| 05 Nov | k1ob_2 (+c) | M2A | triangle 3/11/7 | orbit | pickup | object | 1 | 1 | 1 | 1 | 7.3 (7.3-7.3) | 31.8 s from his TAKEOFF | drop: hit the net |
| 05 Nov | nr9m | M2A | triangle 3/11/7 | hover | re-attach | plate 9 | 9 | 9 | 0 | 0 | 6.7 (3.1-9.0) | - |  |

Registry ids are RIG-1104-<flight> and RIG-1105-<flight>; "(+c)" marks a correction row (RIG-1104-k1m_3c, RIG-1105-k1ob_2c). Flights that were launched but never armed are left out: nr1, nr1_3, fm9, fm9_3, nr1r_2, sq9, nr9o, nr9o_3, k1ob and k1ob_3. For sq8_2 the catch was proven 14.3 s after the release and the fleet resized to four at 14.4 s (the registry's 14.4 s).

## Every plate descent

| day | flight | target | # | from | outcome | height | miss | radial | tang. | body rad. | swing rad. | his ref rad. | body moved rad. last 1 s | lowest: miss | lowest height | contact line xy | ring tilt deg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 04 | sq6b | plate 9 | 1 | match | aborted | never | 9.3 | -9.2 | -1.7 | -4.7 | -4.5 | -0.5 | -10.4 | 9.3 | +26.1 | - | 4 |
| 04 | sq6m | plate 9 | 1 | match | contact not proven | plate | 2.8 | -2.2 | +1.8 | -0.3 | -1.9 | +0.4 | -1.8 | 2.8 | +0.3 | 2.8 | 1 |
| 04 | sq6m | plate 9 | 2 | after proof | caught+proven | plate | 1.9 | -1.5 | +1.2 | +0.6 | -2.1 | +0.3 | +1.5 | 1.9 | +0.1 | 1.9 | 3 |
| 04 | nr1_2 | plate 1 | 1 | match | contact not proven | plate | 0.4 | -0.3 | +0.2 | -0.4 | +0.0 | -0.7 | +2.7 | 0.4 | +0.5 | 0.4 | 3 |
| 04 | nr1_2 | plate 1 | 2 | after proof | contact not proven | plate | 0.3 | -0.3 | +0.2 | -0.2 | -0.1 | -0.8 | -3.4 | 0.3 | +0.5 | 0.3 | 2 |
| 04 | nr1_2 | plate 1 | 3 | after proof | contact not proven | plate | 0.4 | -0.3 | +0.1 | +2.6 | -2.9 | -0.8 | -1.3 | 0.4 | +0.5 | 0.3 | 4 |
| 04 | nr9m | plate 9 | 1 | match | miss | plate | 6.2 | -6.1 | +1.2 | -1.8 | -4.3 | -0.0 | +2.2 | 3.1 | -7.9 | - | 8 |
| 04 | nr9m | plate 9 | 2 | match | miss | plate | 7.0 | -6.8 | +1.7 | -1.2 | -5.6 | -0.4 | -0.5 | 3.8 | -9.2 | - | 4 |
| 04 | nr9m | plate 9 | 3 | match | miss | plate | 6.6 | +4.9 | +4.5 | +3.6 | +1.3 | +0.4 | +0.7 | 1.6 | -11.5 | - | 8 |
| 04 | nr9m | plate 9 | 4 | match | aborted | never | 1.8 | +0.2 | +1.8 | +3.9 | -3.7 | +1.1 | -0.3 | 1.8 | +1.1 | - | 4 |
| 04 | nr9m_4 | plate 9 | 1 | match | contact not proven | never | 3.0 | +2.3 | -1.8 | +0.4 | +2.0 | +1.1 | +3.2 | 3.0 | +3.8 | 3.0 | 3 |
| 04 | nr9m_4 | plate 9 | 2 | after proof | miss | plate | 9.7 | -9.6 | -1.3 | -3.3 | -6.3 | +0.9 | +0.2 | 8.0 | -0.9 | - | 2 |
| 04 | nr9m_4 | plate 9 | 3 | match | contact not proven | never | 2.4 | -1.9 | -1.5 | -0.1 | -1.8 | +1.0 | +0.3 | 2.4 | +1.5 | 2.4 | 1 |
| 04 | nr9m_4 | plate 9 | 4 | after proof | miss | plate | 8.0 | +5.8 | -5.4 | +3.4 | +2.4 | +1.0 | +3.1 | 6.4 | -2.4 | - | 1 |
| 04 | nr9m_4 | plate 9 | 5 | match | contact not proven | plate | 5.9 | +5.8 | -1.3 | +5.1 | +0.6 | +0.9 | +1.6 | 5.9 | +0.4 | 5.9 | 0 |
| 04 | nr9m_5 | plate 9 | 1 | match | contact not proven | plate | 6.2 | +5.1 | +3.6 | +0.1 | +5.0 | +0.3 | +0.2 | 4.8 | -0.5 | 4.8 | 4 |
| 04 | nr9m_5 | plate 9 | 2 | after proof | miss | plate | 6.1 | -6.1 | +0.7 | -2.8 | -3.2 | +0.7 | -1.3 | 7.8 | -3.2 | - | 2 |
| 04 | nr9m_5 | plate 9 | 3 | match | contact not proven | plate | 5.4 | +4.9 | -2.1 | +1.1 | +3.8 | +0.4 | +3.3 | 5.4 | +0.2 | 4.9 | 2 |
| 04 | nr9m_5 | plate 9 | 4 | after proof | miss | plate | 6.1 | +6.0 | +0.7 | +5.3 | +0.7 | +0.5 | +3.4 | 8.4 | -2.4 | - | 5 |
| 04 | nr9m_5 | plate 9 | 5 | match | contact not proven | plate | 7.6 | +4.7 | -6.0 | +6.8 | -2.0 | +0.5 | +3.6 | 8.8 | -0.6 | 5.1 | 5 |
| 05 | fm9y_2 | plate 9 | 1 | match | miss | plate | 5.7 | -4.7 | +3.1 | -2.0 | -2.8 | -0.8 | -1.3 | 5.7 | +0.0 | - | 1 |
| 05 | fm9y_2 | plate 9 | 2 | match | miss | plate | 10.7 | -10.0 | -3.6 | -3.0 | -7.0 | +1.8 | -3.8 | 9.3 | -2.6 | - | 3 |
| 05 | fm9y_2 | plate 9 | 3 | match | miss | plate | 5.1 | +4.7 | -2.0 | +1.6 | +3.1 | +1.2 | -1.8 | 16.5 | -2.5 | - | 2 |
| 05 | nr1r_3 | plate 1 | 1 | match | contact not proven | never | 2.3 | -1.2 | +1.9 | -0.9 | -0.3 | -1.0 | +1.2 | 2.3 | +2.2 | 2.3 | 2 |
| 05 | nr1r_3 | plate 1 | 2 | after proof | miss | plate | 8.2 | -4.3 | -7.0 | -3.8 | -0.5 | -1.0 | -1.4 | 6.9 | -3.3 | - | 4 |
| 05 | nr1r_3 | plate 1 | 3 | match | contact not proven | never | 0.9 | -0.8 | -0.4 | -2.4 | +1.6 | -0.8 | +2.2 | 0.9 | +1.6 | 0.8 | 1 |
| 05 | nr1r_3 | plate 1 | 4 | after proof | miss | plate | 11.0 | +10.8 | -1.7 | +4.5 | +6.3 | +0.1 | +6.0 | 5.8 | -2.6 | - | 2 |
| 05 | nr1r_3 | plate 1 | 5 | match | miss | plate | 9.1 | -8.9 | +1.7 | -3.6 | -5.3 | -0.6 | +0.6 | 8.4 | -2.2 | - | 5 |
| 05 | nr1r_3 | plate 1 | 6 | match | miss | plate | 3.4 | +0.8 | +3.3 | -0.4 | +1.2 | -1.0 | +2.2 | 10.9 | -3.6 | - | 3 |
| 05 | nr1r_3 | plate 1 | 7 | match | miss | plate | 3.4 | -0.6 | -3.4 | -4.2 | +3.6 | -3.7 | +1.4 | 8.5 | -2.2 | - | 3 |
| 05 | nr1r_3 | plate 1 | 8 | match | aborted | never | 15.7 | -15.6 | +1.9 | -8.1 | -7.5 | -3.8 | -4.5 | 15.7 | +3.8 | - | 2 |
| 05 | sq8_2 | plate 8 | 1 | match | aborted | never | 17.9 | -16.4 | +7.2 | -10.7 | -5.7 | -0.4 | -13.2 | 17.9 | +13.1 | - | 13 |
| 05 | sq8_2 | plate 8 | 2 | match | caught+proven | plate | 1.6 | -1.6 | +0.3 | +4.1 | -5.6 | -0.2 | +3.1 | 1.6 | -0.1 | 1.7 | 28 |
| 05 | nr9o_2 | plate 9 | 1 | match | miss | plate | 8.3 | +5.0 | -6.6 | +2.9 | +2.1 | +0.2 | +1.6 | 7.4 | -4.9 | - | 2 |
| 05 | nr9o_2 | plate 9 | 2 | match | aborted | never | 3.8 | -2.6 | +2.8 | -2.3 | -0.3 | +0.5 | +1.9 | 3.8 | +13.3 | - | 6 |
| 05 | nr9o_2 | plate 9 | 3 | match | aborted | never | 4.9 | -4.9 | -0.0 | -1.0 | -3.9 | +0.4 | +3.8 | 4.9 | +14.0 | - | 2 |
| 05 | nr9o_2 | plate 9 | 4 | match | miss | plate | 8.9 | +6.1 | -6.4 | +3.2 | +2.9 | +0.2 | +3.2 | 8.2 | -5.1 | - | 3 |
| 05 | nr9m | plate 9 | 1 | match | miss | plate | 6.2 | +5.6 | -2.7 | +3.3 | +2.3 | +2.7 | +1.9 | 6.0 | -2.4 | - | 4 |
| 05 | nr9m | plate 9 | 2 | match | miss | plate | 3.9 | -2.7 | -2.8 | -0.1 | -2.6 | +2.6 | -4.9 | 10.2 | -0.3 | - | 3 |
| 05 | nr9m | plate 9 | 3 | match | miss | plate | 5.5 | +3.2 | +4.4 | +6.4 | -3.2 | +2.9 | +0.5 | 10.7 | -4.6 | - | 1 |
| 05 | nr9m | plate 9 | 4 | match | miss | plate | 8.0 | -7.0 | -3.8 | -3.7 | -3.3 | +2.5 | -7.3 | 7.5 | -2.9 | - | 3 |
| 05 | nr9m | plate 9 | 5 | match | miss | plate | 7.5 | -6.3 | +4.1 | -3.9 | -2.3 | +2.2 | -0.8 | 7.7 | -1.8 | - | 2 |
| 05 | nr9m | plate 9 | 6 | match | miss | plate | 3.1 | +1.6 | -2.6 | +1.2 | +0.4 | +2.7 | -0.6 | 10.8 | -5.5 | - | 3 |
| 05 | nr9m | plate 9 | 7 | match | miss | plate | 7.1 | +6.7 | +2.4 | +5.6 | +1.1 | +2.8 | +0.3 | 6.6 | -2.5 | - | 1 |
| 05 | nr9m | plate 9 | 8 | match | miss | plate | 9.0 | -1.8 | -8.8 | -1.7 | -0.0 | +2.0 | +1.1 | 8.8 | -3.1 | - | 3 |
| 05 | nr9m | plate 9 | 9 | match | miss | plate | 6.7 | -6.4 | -1.8 | -2.9 | -3.5 | +1.9 | +4.3 | 7.5 | -2.4 | - | 1 |

All lengths in cm. "height": plate = at plate height, never = never within 1 cm (read at the lowest point). "lowest height" is the tip's lowest height above the weld point in the descent; a negative value means it sank past the plate.

## Reproduction of the 5 Nov triangle numbers (notes_for_tejen_2026-11-05.md, question 1)

| quantity | notes (5 Nov) | this table: at plate height | this table: lowest point |
|---|---|---|---|
| descents that reached plate height (of all descents) | 16 | 16 (of 21) | same |
| radial miss, cm | -7.8 to +9.5 | -8.9 to +10.8 | -8.1 to +10.5 |
| tangential miss, cm | mostly within 4, two at 9-10 | 10 of 16 within 4; beyond: 4.1, 4.4, 6.4, 6.6, 7.0, 8.8 | 10 of 16 within 4; beyond: 4.0, 4.1, 5.7, 6.7, 7.4, 9.6 |
| miss xy, cm | 3.5 to 10 (11 Nov sheet) | 3.1 to 11.0 | 5.8 to 10.9 |
| QUAD4 body radial, cm | -4 to +11 | -4.2 to +6.4 | -5.0 to +9.1 |
| swing radial (tip - body), cm | -10 to +7 | -5.3 to +6.3 | -9.4 to +6.5 |
| body radial move in the last 1 s, cm (size) | 3 to 10 | 0.3 to 7.3 | - |
| radial sign changes between consecutive descents of a flight | alternating | 9 of 13 | 9 of 13 |
| sq8_2 catch: tip off the plate centre, cm | 1.7 | 1.6 (at the contact: 1.7) | 1.6 |
| sq8_2 catch: body radial, cm | 7 inward (-7) | +4.1 (at the contact: +2.1) | +3.9 |
| sq8_2 catch: swing radial, cm | 6 outward (+6) | -5.6 (at the contact: -3.6) | -5.4 |

**Agreement.** The count agrees: the same 16 of 21 descents reached plate height. The other five are nr9o_2 #2-#3 and nr1r_3 #8, aborted 3.8-14 cm above the plate, and nr1r_3 #1 and #3, contacts whose magnet stopped 1.6-2.3 cm above the plate. The radial extremes agree within about 1 cm (at plate height -8.9 to +10.8, lowest point -8.1 to +10.5, notes -7.8 to +9.5). The swing range agrees at the lowest point (-9.4 to +6.5 here, -10 to +7 in the notes). The radial sign changed in 9 of 13 consecutive pairs, so the alternation holds mostly.

**Partial disagreement.** The tangential spread is wider than "mostly within 4, two at 9-10": 10 of 16 are within 4 cm at plate height (10 at the lowest point), and the rest are 4-10 cm. The body's radial range is narrower: -4.2 to +6.4 at plate height and -5.0 to +9.1 at the lowest point, against -4 to +11. "Moved 3-10 cm radially in the last second" does not hold: the size of that move is 0.3-7.3 cm (median 1.5). The script behind the notes was not found, so the instant it read is not known. Its numbers sit closest to this table's lowest point (where the magnet ended after sliding off), not to the instant the tip reached the plate's height. The qualitative reading stands: the misses are mostly radial, both inward and outward, and the body and the swing each give several cm.

**Disagreement: sq8_2's catch.** The tip offset agrees (1.6 cm at plate height, 1.7 cm at the contact and in the contact line and notes). The split between body and swing does not. Here the body is 2-4 cm OUTWARD and the swing 4-6 cm INWARD, at the contact and at plate height. The notes have 7 cm inward and 6 cm outward. This was checked a second way, from the raw poses at the CONTACT line's time: horizontally from the ring centre the body sits at 24.6 cm, the plate at 22.5 cm and the tip at 21.1 cm. At that instant the ring leaned 28-29 deg (the T), so the split depends strongly on the frame. In the tilted ring frame the body 1 m up reads about 48 cm "inward". The notes' split may come from another frame or instant. This table's numbers are horizontal.

## What the plate misses have in common

|  | misses (at plate height, > 2.75 cm off) | on the plate (<= 2.75 cm) |
|---|---|---|
| descents | 31 | 5 |
| radial larger than tangential | 21 | 5 |
| miss xy, cm: median (range) | 6.6 (2.8-11.0) | 0.4 (0.3-1.9) |
| |radial|, cm: median (range) | 5.6 (0.6-10.8) | 0.3 (0.3-1.6) |
| |tangential|, cm: median (range) | 2.8 (0.7-8.8) | 0.2 (0.1-1.2) |
| radial outward / inward | 16 / 15 | 0 / 5 |
| his reference radial, cm (range) | -3.7 to +2.9 | -0.8 to +0.3 |
| body minus his reference, radial, cm (range) | -6.2 to +6.2 | +0.3 to +4.3 |
| |body radial|, cm: median (range) | 3.2 (0.1-6.8) | 0.6 (0.2-4.1) |
| |swing radial|, cm: median (range) | 2.8 (0.0-7.0) | 2.1 (0.0-5.6) |
| body and swing on the same side (radial) | 27 of 31 | 1 of 5 |
| var(body radial) / var(tip radial) | 0.34 | - |
| var(swing radial) / var(tip radial) | 0.34 | - |
| corr(body radial, swing radial) | +0.47 | -0.97 |
| QUAD4 speed over the last 1 s, m/s: median (range) | 0.05 (0.03-0.11) | 0.04 (0.03-0.07) |
| tip speed against the plate at the reading, m/s | 0.10 (0.01-0.29) | 0.00 (0.00-0.04) |
| ring tilt at the reading, deg | 2.6 (0.2-8.1) | 3.2 (2.0-28.1) |
| plate speed at the reading, m/s | 0.03 (0.00-0.08) | 0.01 (0.00-0.03) |

Plain notes (plate descents; the numbers are the tables' above):
1. **The miss is mostly radial and goes both ways.** Of 31 descents that reached plate height more than 2.75 cm from the centre, 21 were more radial than tangential: 16 outward and 15 inward. They came down 2.8-11.0 cm off, against plates 5.5 cm across. 20 descents ended with his "the tip sunk below the plate (off its edge)".
2. **His reference was not off; QUAD4 was off his reference.** At the reading his reference sat 0.0-3.7 cm radially from the plate, and QUAD4's body 0.2-6.2 cm from his reference.
3. **Body and swing are each a few cm, and in the misses they add.** In 27 of 31 misses QUAD4's body offset and the swing (magnet under the body) were on the same side (radial correlation +0.47). Each alone is a few cm (|body| and |swing| medians above), and each alone holds about a third of the tip's radial variance; the rest is their covariance. Where the magnet came down on the plate the two were small or opposite (tip = body + swing, so this follows): in sq8_2 the body was 4.1 cm out and the swing 5.6 cm in. Trimming one of them alone would not have put the misses on the plate.
4. **Slow, and the ring was still.** QUAD4 moved at 0.05 (0.03-0.11) m/s over the last second, and the tip at 0.10 (0.01-0.29) m/s against the plate at the reading. Plate speed was 0.00 to 0.08 m/s, including the orbit flight nr9o_2, whose circle was paused for the attach. Ring tilt was 0.2 to 8.1 deg.
5. **The square vs the triangle.** The square (sq8_2) had 1 descent at plate height and caught it at 1.6 cm, with the ring leaning 29 deg (the T). The triangle had 35 at plate height, 4 of them on the plate, all on 4 Nov (sq6m's catch and nr1_2's three on plate 1). On 5 Nov it had 19 at plate height and 0 on the plate. One square descent is too few to call the layout the cause.
6. **Plate 1 on 4 Nov (nr1_2) was dead centre:** 3 contacts 0.3 to 0.4 cm off. The catch was real (the plate rose 18-19 mm) but not proven: our 3 mm taut margin rejected it (registry). On 5 Nov, plate 1 (nr1r_3, contact_dz -0.01) stopped 1.6-2.3 cm above the plate twice, then slid off the edge on the later descents.
7. **Contacts that did not hold (4 Nov, plate 9).** 6 contacts were declared with the loosened 60 mm contact radius, 2.4 to 5.9 cm from the centre. Two of them were 1.6 and 3.7 cm above the weld point (nr9m_4 #1 and #3). Every one was "lost" in the proof: the magnet was on the plate's edge or above it. They count as contacts in the tables.

## Pickups (the object on the floor)

| day | flight | ring | # | outcome | detail | height | tip x, y | miss | miss, mean at the bottom | his xy | body x, y | swing x, y | QUAD4 speed 1 s, m/s | object rose, m | his TAKEOFF to lifted, s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 04 | k1m | hover | 1 | caught, not proven | attached, no lift logged; the object rose 1.34 m with the magnet | target | +1.9, -1.5 | 2.4 | 2.1 | 2.5 | +2.6, -2.1 | -0.6, +0.6 | 0.02 | 1.34 | - |
| 04 | k1m_2 | hover | 1 | miss | contact wait made no progress for 10s | target | -0.4, -12.9 | 12.9 | 11.7 | - | +1.4, -6.9 | -1.8, -5.9 | 0.02 | 0.00 | - |
| 04 | k1m_2 | hover | 2 | caught+proven | object lifted 0.08 m | target | -1.3, +2.7 | 3.0 | 3.3 | 2.9 | +0.7, +0.6 | -2.0, +2.1 | 0.07 | 1.37 | 47.1 |
| 04 | k1m_3 | hover | 1 | contact not proven | object attached but the tip 0.043 m off target | target | +2.4, +3.7 | 4.4 | 4.5 | 4.3 | +1.0, +1.0 | +1.3, +2.6 | 0.03 | -0.00 | - |
| 04 | k1m_3 | hover | 2 | caught+proven | object lifted 0.08 m | target | +3.4, +0.0 | 3.4 | 4.0 | 3.5 | +5.3, +0.0 | -1.9, +0.0 | 0.05 | 1.32 | 46.7 |
| 04 | k1m_4 | hover | 1 | caught+proven | object lifted 0.08 m | target | -2.4, -2.3 | 3.3 | 3.8 | 3.3 | -0.7, -0.4 | -1.7, -1.9 | 0.03 | 1.35 | 20.3 |
| 05 | k1d | hover | 1 | caught+proven | object lifted 1.39 m | target | -0.5, -1.7 | 1.7 | 2.1 | 1.9 | -0.6, +0.2 | +0.1, -1.9 | 0.06 | 1.47 | 29.9 |
| 05 | fm9y_2 | hover | 1 | miss | contact wait made no progress for 10s | target | -9.0, -5.1 | 10.3 | 7.3 | - | -5.2, -1.8 | -3.8, -3.2 | 0.03 | 0.00 | - |
| 05 | fm9y_2 | hover | 2 | contact not proven | object attached but the tip 0.051 m off target | target | +4.1, +5.5 | 6.9 | 5.7 | 5.1 | +4.8, +4.5 | -0.7, +1.1 | 0.03 | 0.00 | - |
| 05 | fm9y_2 | hover | 3 | caught+proven | object lifted 1.42 m | target | +1.4, -1.3 | 1.9 | 1.4 | 1.6 | +3.1, +0.1 | -1.7, -1.5 | 0.02 | 1.55 | 104.4 |
| 05 | k1o_2 | orbit | 1 | contact not proven | object attached but the tip 0.060 m off target | target | +2.6, -4.9 | 5.5 | 7.7 | 6.0 | +2.3, -0.4 | +0.3, -4.5 | 0.03 | 0.00 | - |
| 05 | k1o_2 | orbit | 2 | contact not proven | attached, then: measured pickup proof separation | target | -1.1, -1.7 | 2.0 | 2.3 | 2.1 | -1.4, -3.4 | +0.3, +1.7 | 0.06 | 0.00 | - |
| 05 | k1o_2 | orbit | 3 | contact not proven | object attached but the tip 0.075 m off target | target | -6.1, +0.6 | 6.1 | 7.1 | 7.5 | -6.3, +4.4 | +0.2, -3.8 | 0.04 | 0.00 | - |
| 05 | k1o_2 | orbit | 4 | aborted | take-back (LAND / give-up) | target | -10.1, -1.1 | 10.2 | 7.8 | - | -5.7, +0.8 | -4.5, -1.8 | 0.06 | 0.01 | - |
| 05 | k1ob_2 | orbit | 1 | caught+proven | object lifted 1.37 m | target | -6.2, -3.8 | 7.3 | 5.8 | 3.6 | -0.8, -1.9 | -5.4, -1.9 | 0.04 | 1.47 | 31.8 |

Lengths in cm unless marked. "his xy" is his planner's own number: "object attached with xy error" or "the tip N m off target". "miss, mean at the bottom" is the mean while the tip was within 2 cm of its lowest. k1o_2's object body (6) had been redefined in Motive with its centre several cm off (registry). Its 4 rows, and his "6.0 / 7.5 cm off target", are measured against that shifted centre, not the true one. On k1m (4 Nov) the object rose 1.34 m with the magnet, but his pickup proof never passed (registry: proof too tight), so it counts as caught, not proven. On 4 Nov, "object lifted 0.08 m" is the reason his planner logged when LIFT_OBJECT ended; the object then rose about 1.3 m ("object rose").

## Drops into the net

| day | flight | ring | his log: magnet to drop point, cm | tip to ring centre at the release, cm | tip above the ring, cm | fall to the net, m | impact after, s | impact to ring centre, cm | impact speed, m/s | object to ring centre 2 s later, cm | outcome | note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 05 | k1d | hover | 8.5 | 8.4 | 53 | 0.58 | 0.43 | 12.2 | 3.2 | 1.8 | hit the net |  |
| 05 | fm9y_2 | hover | 9.2 | 9.1 | 57 | 0.59 | 0.46 | 6.3 | 3.0 | 2.2 | hit the net |  |
| 05 | k1ob_2 | orbit | 5.9 | 5.8 | 56 | 0.57 | 0.41 | 13.3 | 3.1 | 51.5 | hit the net | the ring was dropped 0.6 s after the release |

The impact is where the object's fall relative to the ring stops: the relative vertical speed comes back above -0.3 m/s after falling faster than -1 m/s. F051 and RIG-1105-k1ob_2c give k1d 11 cm and k1ob_2 14 cm at ~2.8 m/s. This table gives k1d 12.2 cm at 3.2 m/s, fm9y_2 6.3 cm at 3.0 m/s, k1ob_2 13.3 cm at 3.1 m/s. These agree within the definition of the impact instant. In k1ob_2 the object is 51 cm from the ring centre 2 s later because the ring was dropped after the impact.

## Geometry per flight (flights with descents, and welded starts)

| day | flight | launch config | ring (registry) | target weld point xy (ring frame, m) | target z (m) | start plate (log) | tip angle at start (cw deg) | start: tip xy off the start plate's point (mm) | start: tip z off (mm) | tip samples | object samples | note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 04 | sq6b | 1.15 kg config (r 0.204) | 1.15 kg | -0.0050,0.1970 | 0.0165 | - | - | - | - | 45437 | 45595 |  |
| 04 | sq6m | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | - | - | - | - | 16115 | 16218 |  |
| 04 | nr9 | 1.15 kg config (r 0.204) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | 9 | 292.5 | 1682.1 | -24.1 | 33388 | 33655 |  |
| 04 | nr1_2 | 1.15 kg config (r 0.204) | M2A 0.86 kg | 0.1947,-0.1169 | 0.0365 | 9 | 270.1 | 4.7 | 12.0 | 14258 | 14330 |  |
| 04 | nr9m | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | 9 | 268.6 | 74.4 | -28.7 | 20900 | 21014 |  |
| 04 | nr9m_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | 9 | 267.2 | 1011.3 | -28.5 | 13478 | 13551 |  |
| 04 | nr9m_3 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | 7 | 215.5 | 68.9 | 775.6 | 22723 | 22819 |  |
| 04 | nr9m_4 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | 9 | 269.5 | 22.4 | 11.6 | 31891 | 32038 |  |
| 04 | nr9m_5 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | 9 | 270.4 | 3.8 | 13.6 | 21043 | 21136 |  |
| 04 | k1m | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | - | - | - | - | 13894 | 13961 |  |
| 04 | k1m_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | - | - | - | - | 29648 | 29794 |  |
| 04 | k1m_3 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | - | - | - | - | 24547 | 24673 |  |
| 04 | k1m_4 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0039,0.2271 | 0.0365 | - | - | - | - | 32689 | 32844 |  |
| 05 | k1d | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | - | - | - | - | 43492 | 44547 |  |
| 05 | fm9_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 9 | 271.0 | 4.1 | 1.5 | 7061 | 7303 |  |
| 05 | fm9_4 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 9 | 269.7 | 4.7 | -0.4 | 4586 | 4857 |  |
| 05 | fm9s | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 9 | 269.8 | 4.6 | -0.4 | 3983 | 4119 |  |
| 05 | fm9y | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 10 | 290.1 | 948.9 | -50.2 | 6251 | 6456 | typed 4 Nov QUAD4 values (W 0.0365, (0.0039, 0.2271)); scored against 5 Nov W/X/Y |
| 05 | fm9y_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 9 | 270.2 | 1.2 | 0.5 | 31596 | 32592 | typed 4 Nov QUAD4 values (W 0.0365, (0.0039, 0.2271)); scored against 5 Nov W/X/Y |
| 05 | nr1r | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.1986,-0.1147 | 0.0491 | 10 | 302.0 | 744.7 | -53.3 | 16426 | 0 |  |
| 05 | nr1r_3 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.1986,-0.1147 | 0.0491 | 9 | 269.8 | 0.9 | -0.5 | 11810 | 0 |  |
| 05 | sq8 | 0.86 kg config (r 0.225) | M2A 0.86 kg | -0.1146,0.1986 | 0.0491 | 9 | 239.9 | 117.7 | 1.5 | 3577 | 0 |  |
| 05 | sq8_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | -0.1146,0.1986 | 0.0491 | 8 | 239.9 | 5.6 | 1.6 | 8266 | 0 |  |
| 05 | nr9o_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 9 | 268.7 | 5.4 | -0.3 | 35712 | 0 |  |
| 05 | k1o_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | - | - | - | - | 21702 | 22539 |  |
| 05 | k1ob_2 | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | - | - | - | - | 23045 | 23128 |  |
| 05 | nr9m | 0.86 kg config (r 0.225) | M2A 0.86 kg | 0.0000,0.2293 | 0.0491 | 9 | 269.9 | 2.1 | -0.1 | 26870 | 27037 |  |

## Where this table and the registry or the feed differ

| source | says | this table | why |
|---|---|---|---|
| RIG-1105-nr9o_2, F048 | 4 descents reached plate height, 3.5-8 cm off | 2 of 4 reached it (8.3 to 8.9 cm); #2 and #3 were aborted 13-14 cm above it ("alignment lost in the descent") | The other two were read at their lowest point, 3.8 and 4.9 cm off but 13-14 cm up |
| RIG-1105-nr9m | ~8 descents, off the edge or quiet misses 74-103 mm off | 9 descents; at plate height 3.1 to 9.0 cm, lowest point 6.0 to 10.8 cm | The registry's range matches no single reading here; it is nearest the lowest point |
| RIG-1105-fm9y_2 | re-attach: 3 descents never touched plate 9 (4 Nov W/L with the new body 24: ~2 cm high) | the tip reached plate height on all three (lowest -2.6 to +0.0 cm against 5 Nov's W), 5.1 to 10.7 cm off the centre | Never touched: agreed. "~2 cm high" is not seen: the misses were sideways |
| F040, RIG-1104-nr9m_4/_5 | plate 9 descents ended 24-59 mm off the target | contact lines 23.8-59.0 mm (the same); at plate height 5.4 to 9.7 cm | Different instants: the contact line is where the contact was declared, often after the tip had settled closer; the plate-height reading is where it first reached the plate's height |
| notes_for_tejen (sq8_2) | body 7 cm inward, swing 6 cm outward | body +2 to +4 (outward), swing -4 to -6 (inward) | See the reproduction section: the ring leaned 29 deg; the frame decides the split |
| F051 / RIG-1105-k1ob_2c | k1d hit the net 11 cm off centre; k1ob_2 14 cm at ~2.8 m/s | k1d 12.2 cm at 3.2 m/s; fm9y_2 6.3 cm at 3.0 m/s; k1ob_2 13.3 cm at 3.1 m/s | Agree within the definition of the impact instant |

## What could not be measured, and caveats

1. **The 4 Nov plate height.** 4 Nov flew the typed W 0.0365 m (measured 08:44). The welded starts of nr1_2, nr9m_4 and nr9m_5 sat 12-14 mm above it, close to 5 Nov's 0.0491 (geometry table). But the in-flight contacts sat near the typed value: sq6m's proven catch at +1.3 mm and nr1_2's three at +5 to +6 mm. So the 4 Nov plate height is uncertain by about 1 cm, and 4 Nov is scored against the typed W. This matters for nr9m #4 (4 Nov). It held 40 s at +1.1 cm, 1.8 cm from the centre (registry: +12.1-12.5 mm, outside the +12 mm contact band). It counts as "never at plate height", but it may have been resting on the plate.
2. **fm9y_2** typed 4 Nov's W/X/Y after body 24 was redefined. Its welded start sat 1.2 mm in xy and +0.5 mm in height from 5 Nov's values, so it is scored against those (0.0, 0.2293, 0.0491).
3. **The weld point is the tip marker's welded position, not a measured plate centre.** Any offset between the two is common to all of one day's numbers. Plates 1 and 8 use plate 9's point turned by 30 deg per plate, as the launch does; no weld on plate 1 was measured. sq8_2's welded start on plate 8 sat 5.6 mm from its turned point.
4. **Starts not usable as a check:** nr9 (body 24 tracked QUAD2's magnet), nr9m_2, nr1r and fm9y (the tip about 1 m from the ring centre at the start), sq8 (QUAD4 not on its plate) and nr9m_3 (a bad start). None of these flights has a descent. nr9m (4 Nov) has descents. At its start the tip read 74 mm from plate 9's point (7 cm outward) and 29 mm lower: that is not a magnet seated on the plate. The in-flight rows use the ring's pose and do not depend on the start.
5. **No flight lacked its streams.** Every flight with a descent has the ring, body, tip and reference in its bag. The pickup body (6) was not streamed in nr1r, nr1r_3, sq8, sq8_2 and nr9o_2 (re-attach only; not needed) or in k1o (no descent). Times are the recorder's receive times, so every instant carries a few tens of ms of latency.
6. **sq6b (the 1.15 kg ring):** its only descent was aborted 26 cm above the plate when the ring dropped, so it says nothing about the miss.
7. **Single samples, and which instant.** Each reading is one mocap sample (about 90-100 Hz); at these speeds the tip moves under 1 cm between samples. Which instant is read changes the numbers by a few cm (plate height vs lowest point vs the contact line). The lowest point is not a cleaner miss distance. A magnet hanging below the plate can read horizontally close to it when the ring is tilted: on 4 Nov nr9m #1-#3 the lowest points were 8-11 cm below the plate yet 1.6-3.8 cm from its point, with the ring at 4-8 deg. The plate-height reading is the main one here. Quote the reading used.
8. Exploratory: no card, no bars, no repeats.

