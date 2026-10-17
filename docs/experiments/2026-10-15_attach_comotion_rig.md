# Card: QUAD4's attach on the rig with the 14 Oct fixes (15 Oct 2026, rev 4: the critic's two rounds and Tejen's review)

Owner Wesley. Tree: week5 at the sheet's SHA (rev 2's fixes and the clockwise angles, re-flown in the twin first).
Plan: `docs/plan_after_rig_2026-10-14.md` (WP3, with WP8 and WP2). Sheet: `docs/rig/thu_2026-10-15.md`. Run ids
RIG-1015-*. Plates count clockwise (QUAD4's plate is 9, Tejen's 3; `docs/rig/plate_numbering.md`).

## Question

With Tejen's controller flying QUAD4 over plate 9 (floor_join: from its mat straight to the rejoin, the ring held at
1.0 m by our three carriers), does the attach now complete on the rig: contact, our confirmation, the take-over, the
fold-in to four, a level hold on four and the landing?

## What is flown (one package)

14 Oct gave one contact in five descending flights, too few to test the parts one at a time. They fly together;
the bars say which part failed if one did. A dry hold flies first (below).

| Part | Commit | What |
|---|---|---|
| Confirmation | eacab58 + rev 2-3 | Co-motion: contact in the plate frame (xy <= K, normal -15..+12 mm, <= 0.03 m/s for 0.30 s); proven 1.0 s after his lift with the tip within 6 mm of its contact point and following the plate (drift at most half the plate's motion), the plate moving (5 mm range or 3 mm rise), the tether taut against this flight's free hang (-3 mm), the normal within 10 mm (15 in the twin); failed on separation or loss (45 mm for 0.12 s). Rev 2: the free hang is kept through the hold and retries and is measured with the coil OFF too; the CONTACT line says if the typed fallback was used. Rev 4 (Tejen): no hold spans missing data (a gap over 0.1 s or an invalid sample restarts the contact and the proof holds; counted in /magnet/attach_proof); with his planner the proof waits for his lift-done signal (6 s safety net) |
| Bottom hold | eacab58 + rev 2, 4 | His ATTACH_READY waits at the bottom with the magnet ON until contact; leaves on a gross error (0.35 m/s or 0.12 m for 0.3 s, or the face 20 mm below the plate for 0.12 s); his quiet-miss retry; give-up 45 s. Rev 2: the hold judges the tip against the plate point, not his offset target. Rev 4 (Tejen): in the plate frame from the ring's pose (the face clearance along the plate normal); a quiet miss only while the tip is slow (0.03 m/s, his M2 value) on fresh data, stale data restarting its timer |
| Coil on the climb back | rev 2 | After a descent our span sends OFF while he climbs back and matches, ON again for the next descent |
| Drop | rev 2 | A ring drop releases QUAD4's coil even after a confirmed attach (the locked drop rule) |
| Dry hold | rev 2 | `dry_hold:=true` (default off): the coil gate holds QUAD4's coil OFF; contacts and proofs run and are logged; a proof that passes is logged as a false proof and is never an attach |
| One plate point | eacab58 | His bridge aims at the sheet's X/Y (test 3's catch centre 0.013, 0.210), as our manager |
| Aim | sheet | Option A unless Wesley picks B (decisions.md OPEN 14 Oct): t6h_4's body target with his trim off, `target_alignment_mode:=legacy contact_dx:=0.031 contact_dy:=0.010` |
| Arming, landing | 3297746, dade70d | His ARM first, only refused drones named, DISARM stops an ARM, QUAD4 disarms alone 0.10 m above its rest |
| Console, T2 | a73d9a4, f9d0b72 + rev 2 | READY and the buttons (`tools/console.py`), T2 events only, RViz tip/tether/puck |

## Baseline

14 Oct floor_join (RIG-1014-t6f, t6f2_4, t6g_2, t6h_4, t6h5_3): descents in all five; one contact (t6h_4: the magnet
held plate 9 through three lifts) rejected by the old proof's taut check; no fold-in. Aim A has flown twice (t6h_4:
contact; t6h5_3: closest 2.5 cm, no contact).
Replayed with the rig's values (K 24 mm, taut fallback 0.575, `tools/proof_replay.py`): t6h_4's catch proven 2.81 s
after contact (the tip drifted 1.1 mm while the plate moved 16 mm, the free hang measured); no contact in the others;
no false proof. Twin: R1718 proven and folded in; R1719 three inconclusive proofs (the twin's stiff ring hardly moved:
rev 2 waives plate motion in the twin only, where the catch is the joint made at contact).

## Bars (rev 3, frozen before the first flight)

Scored between flights from each flight's bag and T2 log: the manager's CONTACT / ATTACH / PROOF FAILED / DRY HOLD
lines and `python3 tools/proof_replay.py --catch <run>` (the rules replayed with the rig's values, the magnet following
our bagged commands, his lift-done signal and phase; then per contact the tip's largest excursion from its contact
point in the plate frame, the time taut, the plate's rise, and per climb back whether the plate came up), plus
`tools/attach_window.py`.

0. **Dry hold first** (one flight, `dry_hold:=true`). Counts only once at least one CONTACT was followed by his lift.
   Pass: no `DRY HOLD: FALSE PROOF` line; the ring under 15 deg during the hold. A false proof here stops the
   magnet-ON flights for the day.
1. **No false proof** (magnet ON flights). After a `proven`, the tip stays within 10 mm of its contact point (plate
   frame) until 5 s after the take-over (`Drone 4 mux: ours`), with no loss (20 mm for 0.12 s): the report's
   `BAR 1 FAIL` otherwise, and no more attach flights that day.
2. **No missed catch.** Independent of the rules: a catch is the tip within 10 mm of its contact point while the tether
   is taut (at least this flight's free hang - 3 mm) for at least 0.3 s. A catch the manager did not prove, or a proof
   without one, is the report's `BAR 2 MISMATCH`, with the reason found in the log (a lift-done signal, a stale stream,
   a phase abort, a threshold).
3. **Bottom hold safe.** No climb back with the plate coming up: taut, the tip within 10 mm of its place on the plate
   while QUAD4 is 3 cm up, for 0.3 s (the report's `BAR 3 FAIL`); the ring under 15 deg; no carrier rod lets go.
   Fail: `contact_bottom_hold:=false` for the rest of the day.
4. **The attach** (the question). A fold-in to four (`ATTACH: proven`, `Drone 4 mux: ours`, `ATTACH drone 3 (OCP
   resize)`), then 10 s on four with the ring under 10 deg, then LAND with all four down and disarmed.
   Exposure: a flight counts when the tip spent at least 5 s inside K at the bottom. Pass: at least one fold-in in up
   to four counted aim-A flights. Fail: none in two or more counted flights. Fewer than two counted: not tested.
   Each flight is scored as one of: no contact; contact not proven (with the failed condition); proven, not folded in.
5. **Reported, no bar:** contacts per flight and the time from the first descent to each; quiet misses and retries;
   attach_window per phase (tip error, inside-and-slow, swing, ring tilt); which tether length each proof used.

## Stop rules (with the 14 Oct sheet's)

- A false proof (bars 0 and 1): no more attach flights.
- Ring tilt over 30 deg: the drop rule lands everyone and now releases QUAD4's coil too; no more combined flights.
- A drop before the fold-in: GIVE UP (the drop lands the carriers but does not take QUAD4 back; the locked rule's open
  item).
- After the fold-in, an `UNANNOUNCED DETACH` of drone 3 (a false proof showing in flight): LAND.
- Two arm refusals in a row on one drone: the props-off FC check (console FC column, `docs/rig/fc_request.md`)
  before the next launch.
- At the end, DISARM only when the console says nobody is in the air (an emergency excepted).

## Risks

- The bottom hold keeps the magnet ON at the bottom for up to 45 s while the swinging tip touches the plate. The dry
  hold shows its contacts and exits first; on the climb back the coil is now OFF.
- The bottom hold and the aim are rig-only; the twin (rigid tether, no swing) covers the confirmation, the span and
  the fold-in.
- Aim A rests on two flights (one contact). His MPC stays in FREE_SWING during the proof in M1 (Tejen's review,
  15 Oct): not changed here (his code, his OK first).

## Critic

Round 1 (14 Oct, on rev 1), verbatim:

> I'd rewrite the card before it flies. The bottom hold's own catches would be confirmed against the same typed taut
> length that failed t6h_4 (F1). Separately, there are two gaps where QUAD4's coil cannot be switched OFF, and both
> should be closed before the ring flies (F3, F4).
>
> **The five questions.** (1) Tried before? Partly: contact, his lift, a taut check is what failed t6h_4 three times
> (typed 0.572, in flight 0.566-0.571). The new rules rest on one replayed catch plus twin R1718 (rigid tether, no
> hold). Aim A has flown twice, not once. (2) Simplest explanation not addressed: the tip is seldom on the plate and
> slow (inside K and slow 0-11 %); the hold only helps if the tip settles at the bottom, and no bag shows that.
> (3) Falsifier: bar 4 counts flights that enter the hold, not real exposure; bar 2 compares the rules with
> themselves. (4) What it breaks: the locked drop rule for a welded QUAD4 (F4). (5) Asked for: yes; aim A still needs
> Wesley's word.
>
> **F1 (blocker for the claim):** the free-hang store keeps only the last 2 s with the tip 5 cm above the plate and every
> failed proof empties it, so a contact 2 s into the hold, and every retry, uses the typed fallback (0.570 for 1.0 s).
> The CONTACT line prints the fallback as if measured. Fix: measure in MATCH and keep it for the magnet-ON span; log
> "(fallback)"; a test with 10 s at the bottom.
> **F2 (fix before flying):** proof_replay does not run the same rules (contact radius 0.035 vs K 0.024, no taut
> fallback, a fixed lift time, no phase). Replay with the run's values and the bagged signals; give bar 2 an
> independent catch test.
> **F3 (fix before flying):** every hold exit climbs with the coil ON (our span re-sends ON, the gate ignores his OFF):
> an unregistered edge catch lifts plate 9. Aim A also makes the quiet-miss rule measure the tip against plate +
> (0.031, 0.010). Fix: coil OFF on any exit without a contact; measure the quiet-miss error from the plate point; bar 3:
> no climb with plate 9 rising more than 2 cm.
> **F4 (fix before flying):** after "proven" the manager refuses /magnet/command OFF, the drop's only route to QUAD4's
> coil: a drop leaves QUAD4 tied to a falling ring. Fix: accept the OFF once /fleet/payload_dropped is true, plus a test.
> **F5 (rewrite):** bar 1 cannot be scored by either tool; bar 3's 20 mm-below exit is the planner's own off-the-edge
> exit; bar 4 should count exposure (5 s inside K at the bottom) and score each flight as no contact / not proven /
> proven but not folded in.
> **F6 (fix before flying):** the bench cannot fail the dangerous cases; add a long rest before the catch, a
> non-magnetic spacer case, a raise at contact + 2.5 s; fly one hold with the coil OFF first (a new switch, default off).
> **F7 (notes):** the co-motion thresholds (5 mm range, 3 mm rise) sit below the 6 mm drift allowance, so a tip still in
> the world "moves with the plate"; a gap in the tip stream keeps the contact-dwell buffer; aim A's arithmetic holds
> (exact at ring yaw ~0, ~2 mm off at 5 deg); add t6h5_3 to the baseline.
>
> CRITIC: rewrite the card.

**Response (rev 2):** F1 fixed (the free hang is kept through the hold and retries, measured with the coil OFF too,
the fallback named in the log; tests with 10 s at the bottom and after a failed proof). F2 fixed (the replay uses the
rig's values and, from 15 Oct bags, his lift signal and phase; bar 2 now has the independent catch test). F3 fixed
(the span OFF on the climb back and in MATCH after a descent; the hold measures the tip from the plate point; bar 3 as
proposed). F4 fixed (the manager releases on /fleet/payload_dropped, refuses ON after it; tests). F5 adopted (bars 1, 3,
4 rewritten). F6: the bench adds the long rest and the late raise; the spacer case needs a 5-10 mm non-magnetic spacer
on the rig (if one is at hand); the dry hold switch is built and flies first. F7: the tip must follow the plate (drift
at most half the plate's motion); a gap restarts the contact hold; t6h5_3 added to the baseline. The twin waives plate
motion (R1719).

Round 2 (14 Oct night, on rev 2), verbatim:

> The code fixes hold. The bars and the scoring don't yet test the hypothesis, so this needs fixing before the flights.
>
> **The five questions.** (1) Tried before? Yes: rev 1's rules with fixes; evidence one replayed catch (t6h_4) and the
> twin runs R1718 and R1719 (R1719's registry row missing at the time). (2) Contacts are rare; rev 2 makes the check and
> the coil safer, not contacts likelier; bar 4's exposure rule is honest about this. (3) Bars 1 and 2 only test the
> falsifier if their catch definition can tell a catch from a resting tip, and bar 2's can't. (4) Nothing locked broken;
> one rig threshold changed without being called out. (5) Asked for; aim A still Wesley's call.
>
> **Fixes confirmed in the code:** F1; F3 (no OFF during LIFT_PROOF or a retry; one race, a hold exit firing as the
> contact is reported, aborts the proof and releases: fails safe and logged); F4 (the drop flag latched and published
> before the OFF on both drop paths); F6 (no fold-in in a dry hold, his ON cannot reach the coil).
>
> 1. Bar 2's "independent" catch test counts resting tips (a resting tip rides up with a 5 mm rise; "leaves only after
>    the coil goes OFF" is true of every resting tip now; "within 10 mm of the plate" excludes t6h_4's catch at 23.8 mm).
>    Fix: the tip within 10 mm of its contact point while taut (free hang - 3 mm) for 0.3 s; drop the coil-OFF clause.
> 2. Bar 3's 2 cm rise is inside the ring's own motion (plate 9 sits 1-2.7 cm low under the downwash). Fix: fail when
>    the tip stays within 10 mm of the plate while QUAD4 climbs at least 3 cm.
> 3. No tool scores bars 1-3. Fix: a `--catch` mode in proof_replay (per contact the excursion to 5 s after the
>    take-over, the time taut and the plate rise; per climb whether the tip stayed on the plate).
> 4. The replay still differs from the manager (magnet ON from the first sample; the 2.5 s lift fallback disabled when
>    the bag has his signal). Fix: the magnet from the bagged /magnet/command, the free hang while OFF, keep the fallback.
> 5. `normal_ok` went from 10 to 15 mm on the rig too, unjustified (R1719 try 2 failed on the normal, a twin artefact;
>    the rig's catch read +2.6 mm). Fix: 0.010 on the rig, 0.015 in the twin branch; write R1719's row.
> 6. The dry hold and the bench can pass without testing anything (bar 0 with zero contacts; bench d outside K; bench e
>    needs a spacer). Fix: bar 0 counts after a CONTACT and his lift; run the bench's negatives under `dry_hold:=true`
>    with the magnet on plate 9, taut, moving the ring's edge and QUAD4 up and down together: PROOF FAILED, never a
>    false proof.
> 7. Stop rules missing: a drop before the fold-in does not take QUAD4 back (GIVE UP); after the fold-in an UNANNOUNCED
>    DETACH of drone 3 is a false proof in flight (LAND).
> 8. (note) The gap rule uses arrival time: an executor stall over 0.1 s restarts the 1.0 s dwell and the burst after
>    it inflates the speed; use the header stamps, or log the restarts and the longest gap.
> 9. (note) The twin waiver stays in the twin; the twin re-fly no longer exercises the plate-motion and follow checks;
>    `dry_hold` in the twin still welds at contact.
>
> Verdict: fix first. CRITIC: rewrite the card.

**Response (rev 3):** 1-3 adopted: bars 2 and 3 rewritten as proposed, and `proof_replay.py --catch` scores bars 1-3
between flights (tested on synthetic tracks; on t6h_4 it reads a catch, proven, the tip within 2.0 mm of its contact
point and taut 100 % to 5 s after the proof). 4 fixed (the magnet follows the bagged /magnet/command; the 2.5 s fallback
kept). 5 fixed (rig 10 mm, twin 15 mm; R1719's row written). 6 adopted (bar 0's exposure rule; bench d and e under the
dry hold on plate 9). 7 adopted (two stop rules). 8: the gap rule now restarts only after 0.3 s (a mocap dropout; the 14
Oct stalls were 0.16-0.25 s) and the restarts are counted in /magnet/attach_proof; header stamps not adopted (one clock
for the proof's timers and its samples). 9 noted: the plate-motion and follow checks are tested by unit tests and the
replay, not the twin.

## Tejen's review (15 Oct, of the package with the proposed changes)

His points, as Wesley relayed them, and what changed (rev 4, no bar changed):

| His point | Rev 4 |
|---|---|
| The quiet-miss retry is not "quiet": eligibility ignores contact speed; stale input does not reset its timer | Eligible only while the tip moves at most 0.03 m/s over the plate on fresh data; stale data restarts the timer |
| The proof dwell can span a short invalid-data interval | A gap over 0.1 s or an invalid sample restarts the contact and the proof holds (the critic's round 2 preferred 0.3 s to ride out laptop stalls; Tejen's rule kept: a 0.25 s stall costs itself plus the 1.0 s hold, inside the 3 s budget) |
| The 2.5 s lift fallback can start the proof before his lift finishes | With his planner (which publishes the lift's end) the proof waits for his signal; 6 s only if it never comes |
| The 20 mm below-the-plate exit uses the marker's world z, not the face clearance along the plate normal | The hold reads the ring's pose and measures the tip in the plate frame |
| The package lacked the graph wiring that selects the co-motion rules | The next package carries graph_attach.py, profiles.py and the configs |

His answers: trim learning at 0.20 m/s above contact is a reasonable experiment within the existing correction bounds,
without relaxing the contact or proof speeds (aim B as built); keep the measured lift proof; do not copy M2 controller
modes or change TAKEOFF here. Puck contact height: 37.9 and 36.2 mm against his assumed 34.5 mm (the pickup, later).

Tejen's second review (15 Oct, of the rebuilt package): three points, fixed (rev 5, no bar changed):

| His point | Rev 5 |
|---|---|
| The quiet-miss reset is bypassed by his planner's input-loss return, so stale time can count toward a retry | His timer callback's input-loss returns (both) reset our quiet-miss timer and the tip-speed estimate |
| The 6 s lift fallback still assumes a lift nobody confirmed | With his planner wired, the proof starts only on his lift-done signal (no timed fallback); the manager warns once if the signal is 6 s late |
| A stale configured ring attitude falls back to world axes | It pauses the plate-frame checks instead (the below-the-plate exit, the quiet miss); the gross exits still run |


## Twin note (18 Oct, exploratory join re-fly R2220-R2236, flight code 9d30665)

- **A false proof in the twin (R2228, `m1c_nocatch_hover_2`, `sim_catch` false):** contact tries 1-2 failed (separated,
  inconclusive); try 3 was proven 9 s after the contact: tip drift 4.6 mm, tether 0.5890 m against a free hang of 0.5906
  (taut within 3 mm), the plate moving 0.2 mm. The twin waives plate motion ("no plate motion needed (twin)"), so an
  uncaught magnet resting still on the plate passes. QUAD4 was folded in uncaught and the fleet landed. The rig rule
  needs the plate to move with the tip, plus `proof_taut_len`, so this is not a rig failure. It does mean the twin's
  no-catch arms cannot test a false proof under comotion. The other no-catch hover (R2227) rejected its 3 contacts.
- **Orbit joins (twin only; the rig refuses an orbit under partner_contact):** 3 of 4 dropped the ring (tilt > 30) in
  his approach (R2229-R2231), and R2232 held at 20 deg. Before each drop all three rod tensions rose (3.1 -> 3.5-4.7 N) at
  even throttle, with the planner ticking normally: something pulled the ring down. The cause is open. On 12 Oct, on the
  older tree, the same configs passed (R1694, R1695, R1699).
