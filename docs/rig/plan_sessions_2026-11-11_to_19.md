# The four sessions left (Wed 11, Thu 12, Wed 18, Thu 19 Nov): APPROVED by Wesley 6 Nov ~14:43

> **SUPERSEDED 10 Nov ~23:45 (Wesley's new plan, decisions DECIDED 2026-11-10 23:10 / 23:27 / 23:42):** every session flies ONE list, `docs/rig/wed_2026-11-11.md`, from where the last session stopped (nrtf, nr1f, nro, k1zo, fm9c, fm9hc; backups k1z, fm9, fm9h). rel and relo are dropped. The rest of this file is the 6 Nov plan, kept for its setup notes.

**Why it changed.** The supervisor asks to put the combined paper first (Wesley 6 Nov; its Overleaf clone
`multi_drone_swung_payload_item_collection_wesley_version/main.tex`). Te-Jen already has its path-planning results. From
the rig the paper needs three things (decisions.md, 6 Nov ~14:45):

| Paper section | What is flown | Static (hover) | Moving (the 0.3 m orbit) |
|---|---|---|---|
| Detach performance | the UNANNOUNCED release: QUAD4's magnet switched off, the planner not told (rel) | 5 | 3 (the circle runs on; new script relo) |
| Attach performance | our detach, then Te-Jen's re-attach on plate 9 of the triangle, our weld proof (nr9m, or nrtf with the funnel) | 5 | 3 (the circle stops for the attach, then resumes: nro) |
| Application | the whole mission in one flight on the triangle: detach, pickup, drop into the net, re-attach, land (fm9, the funnel on 9) | 2-3 | 1-2 if hover works |

Each is reported as a success rate, the time to settle and the ring's swing at the event. The triangle stays (Wesley: the
square's catch looked lucky); the funnel is what should make the attach reliable.

## The order of work (a session is ~12-15 armed flights, 8-11 am)

**Wed 11 (S3): the triangle all session, no re-hang.**
1. rel x5: the unannounced release in hover (static detach). Re-catch QUAD4's magnet on plate 9 by hand between flights.
2. nrtf x5 if the funnel is on plate 9 (W re-measured with the magnet in it), else nr9m x5: the static attach.
3. If time: k1z x1 (the pickup and drop at 1.2 m).

**Thu 12 (S4; the thesis's real-world freeze at 11:00).**
1. relo x3: the unannounced release on the orbit (moving detach). New script; a simulation run and a desk check first.
2. nro x3: the stop-and-attach on the orbit (moving attach); the funnel stays on.
3. k1z x2: the pickup and drop (the thesis's K, and the application's middle).
4. If time: fm9 x1.

**Wed 18 (S5).** fm9 x3 (the application in hover, with the funnel); then anything short of 5 static / 3 moving.

**Thu 19 (S6).** fm9 on the orbit x1-2 if hover works; the paper's video takes (a fixed camera for the "ghost" images);
the supervisor's demo; any gaps.

## What this drops or moves (your call)
- **The 1 m baselines** (h3/f3/ls3 on the triangle, h4/ls4/f4 on the square: 12 flights and two re-hangs) and the square
  re-attach claim (nrs x3) leave the plan. The thesis's real-world trajectory results then rest on the flights it has
  (0.55 m cables, and the 1.15 kg ring on 29 Oct).
- The cable re-read (h3rr) leaves too.
- Wednesday's sheet as written (claim on the square first, baselines, rel last) is replaced by the order above.

## Needed before Wednesday (the lab chat)
- relo (the release on the orbit) and an fm9 script with today's settings (takeoff gate, contact_dz -0.03, the 55 deg drop
  and 15 cm loss check, the small object, the pickup at 1.2 m as k1z): written, desk-checked, one simulation run each.
- QUAD4's drop rule (being built and tested apart from the batch; merged after it, before Wednesday).
- The Wednesday and Thursday sheets rewritten in this order, desk check re-run.
- Wesley: the funnel printed and fitted on plate 9 (v4, TPU).

## Risks
- The triangle's re-attach caught 0 of 19 on 5 Nov without the funnel; 5 static attaches may need many tries. Fallback:
  attach attempts only, each scored (miss distance, F052's table), and the square on Thu 19.
- Every rel flight needs QUAD4's magnet re-caught on plate 9 and the cable re-hung (~5 min each).

## Added on approval (Wesley 6 Nov ~14:43)
- If the pickup struggles (the other quads' downwash in the cage), the drop cases start with the object already on
  QUAD4's magnet: an fm9 variant on his planner's `delivery_rejoin` mode (fm9h), scripted beside fm9.
- Ideas to help the attach (hardware or otherwise): worked up by the lab chat, ranked by effort for these sessions.
