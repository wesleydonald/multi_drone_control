# Single-run figures of the combined paper (real flights)

`paper_figs.py` draws the three single-run figures that `main.tex` ("Experiments and results") asks for, from one real
flight each, so the same commands re-draw them from the flights of 11, 12, 18 and 19 Nov:

| Function | Paper figure | Example flight | Output (in `figs/`) | Size |
|---|---|---|---|---|
| `attach_run` | "Attach results over a single run" | `2026-11-05/sq8_2` (the square: release, his take-off, contact on try 1, proven, fold-in) | `attach_2026-11-05_sq8_2.pdf/.png` | 3.5 x 5.5 in (one column) |
| `detach_run` | "Detach results over a single run" | `2026-11-05/nr9m` (planned detach, a stand-in until the unannounced `rel` releases) | `detach_2026-11-05_nr9m.pdf/.png` | 3.5 x 4.9 in (one column) |
| `application_run` | "Real world Top down trajectory" (the application) | `2026-11-05/fm9y_2` (detach, pickup on try 3, transit, drop into the net, three re-attach descents that missed) | `application_2026-11-05_fm9y_2.pdf/.png` | 7.16 x 3.5 in (the page) |

Use the PDF in LaTeX at its size (`\includegraphics{...}` with no scale: `figure` for one column, `figure*` for the page).
The text is Times (newtx) to match IEEEtran; no run ids are drawn (the file name carries the flight).

## Re-run on a new flight

```bash
source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
cd ~/multi_drone_control
nice -n 10 python3 results/analysis/paper_figs_2026-11/paper_figs.py attach      results/rig/<day>/<flight>
nice -n 10 python3 results/analysis/paper_figs_2026-11/paper_figs.py detach      results/rig/<day>/<flight> [--event N]
nice -n 10 python3 results/analysis/paper_figs_2026-11/paper_figs.py application results/rig/<day>/<flight>
```
- The flight can also be given as `<day>/<flight>`. Options: `--out <dir>` (default `figs/` here), `--width single|double`
  (swap the layout: one column with the plan view on top, or the page with the plan view on the left; the defaults are
  the ones in the table), `--event N` (detach: the N-th release of the flight, as `paper_rig.py` lists them; 0 = the first).
- Which flights: `attach` for a re-attach (QUAD4 welded at the start) or an attach from the mat; `detach` for any flight
  with a release (an operator's OFF on any quad's magnet, or our planned detach of QUAD4); `application` for
  `pickup_*` flights (with QUAD4 welded: the full application; from the mat: pickup and drop only).
- The first read of a bag takes 0.5-1 min (QUAD4, the ring, the tip and the object through `joins.read_bag`, cached in
  `~/mdc_overnight/joins_cache`; every quad's pose cached in `~/mdc_overnight/paper_figs_cache`); after that a figure
  takes a few seconds. The bags of 5 Nov and the tables' flights are already cached. Like `paper_rig.py`: run it with
  the rig stack down, and not in the first 3 min of a laptop Gazebo run.
- Everything comes from the flight's files: events from `<flight>.log` (the same parsers as `paper_rig.py` and
  `joins.py`), poses from the bag, the ring's set-point from the drone-0 tracker log. The definitions are
  `paper_rig.py`'s, so a figure and its table row agree (checked: nr9m's tilt peak 7.7 deg and 1.7 cm dip; r303_4's
  detection at +0.42 s).

## What each panel shows

Colours: the ring black, the other quadrotors grey, QUAD4 (the returning quadrotor) red as in RViz, its magnet dark red
(track solid, height dashed), the object purple. A released quadrotor other than QUAD4 takes its own RViz colour.

### Attach (`attach_run`): time in s since the fleet's TAKEOFF (the clock of `events.csv` and `phases.csv`)
- **(a) Plan view**, from the release to 1 s after the fold-in: the ring and its 12 plates at the contact (QUAD4's
  plate filled red), the other quadrotors' tracks with their place at the contact (grey dots), QUAD4's body (red; open
  circle at the release, dot at the contact) and its magnet (dark red).
- **(b) The magnet's last approach** in the plate's frame (radial outward, tangential anticlockwise, cm), from his first
  MATCH to the proof: the plate (55 mm disc) and the contact (x). Misses show here as passes beside the disc.
- **(c) QUAD4's state**: Carrier (on the ring) until our release; Released (held by our planner) until his take-off;
  Take-off until the hand-over (PARTNER RELEASE) or his next phase, whichever is later; then his planner's phases:
  Approach, Match, Descent (the descent and its bottom hold), Proof (from the contact); Attached from the fold-in
  (FLEET RESIZED back to 4 after ATTACH: proven); Land from LAND. Every retry shows as a jump back up. A flight from
  the mat starts in Ground.
- **(d) Offset from plate**: the magnet's horizontal distance (solid) and height above the plate (dashed), cm. The
  scale stops at 45 cm: the step-out after the release (about 1 m) runs off the top.
- **(e) Tilt** of the ring (normal from vertical, deg), **(f) Position error** of the ring (3-D distance to the planner's
  set-point, cm).
- Dotted lines through (d)-(f): the release, the contact that was proven (the last contact if none was), the fold-in,
  LAND. Window: 2 s before the release (before his take-off from the mat) to 2 s after LAND, cut at a loss
  (sq8_2 ends 1.2 s after LAND, at the false loss that dropped the ring during the landing).

### Detach (`detach_run`): t = 0 at the release (the operator's OFF, or our OFF after the 3 s unload)
- **(a) Plan view** over the window: the ring at the release, the other quadrotors' tracks (dot: at the release), the
  detached quadrotor from the release on (and its magnet if it is QUAD4).
- **(b) Horizontal shift** of the ring (black) and the other quadrotors (grey) after the release, cm, from their mean
  place over the "before" window (+ = where they started).
- **(c) Tilt** of the ring, **(d) Height change** of the ring (dip: below 0), **(e) Quadrotor shift**: each other
  quadrotor's 3-D distance from its mean place before, cm.
- Marked: the release (solid line), the detection (dashed: the planner's UNANNOUNCED DETACH line, unannounced releases
  only), the unload (grey band, planned detaches only).
- Window: `paper_rig.py`'s. It starts at its "before" window (5 s, cut where the ring's set-point last moved, so the
  lift's tail stays out) and ends at the first of 15 s, LAND, a loss, the next resize, or QUAD4's next descent.
- Shifts are measured relative to the ring's set-point (interpolated from the tracker log): in a hover that is the
  plain shift. On a moving ring (`relo`) the lag behind the set-point counts as shift; the paper's path deviation is
  clock-free, so read the lag apart (nr9o_2: the ring and the quadrotors shifted about 15 cm from the set-point
  after the trajectory hold at the detach).

### Application (`application_run`): time in s since the fleet's TAKEOFF
- **(a) Plan view** from QUAD4's release to LAND: the ring at the drop, the other quadrotors, QUAD4's body (open
  circle: the release), the object from where it lay (square) until it lay in the net.
- **(b) Mission state** (`paper_rig.py`'s phases): Lift (TAKEOFF to the ring's set-point at the top), Detach (our UNLOAD
  to the hand-over), Pickup (to "object lifted"; from his TAKEOFF for a pickup from the mat), Transit (to DROP_OBJECT),
  Drop (to CLEAR_DROP_ZONE), Re-attach (to the fold-in, or to LAND when it never folded in), Attached, Land (to
  "Landed - disarming"). Hover fills a wait (after the lift, or after a pickup-and-drop's drop).
- **(c) Heights**: QUAD4's body (red), its magnet (dashed), the object (purple), the ring (black), m. The pickup tries
  and the re-attach descents show as the magnet's dips.
- **(d) Tilt**, **(e) Position error** of the ring. Dotted lines: every state transition. The scales ignore the ring on
  the floor after LAND. Window: 1 s before TAKEOFF to 1 s after the fleet landed, cut at a loss.

## What the data could not show (and what was done about it)
- **The object after the drop** (fm9y_2): its Motive body snaps onto the magnet's markers whenever the magnet passes
  over the net (three jumps, of up to 0.4 m, to within 1.6-3.4 cm of the tip). The object is drawn until 3 s after
  the drop (`OBJ_AFTER_DROP_S`); it lay in the net from then on.
- **The ring after a drop**: its pose can freeze (fm9_2 held 0.6 m and 60 deg after the drop), so every window ends at
  the first loss.
- **No detection time yet**: nr9m is a planned detach (our own OFF), so its figure has the unload and the release but
  no detection. The unannounced path was checked on the 8 Oct release r303_4 (0.55 m links; context only).
- **The hand-over**: in a re-attach, his take-off comes while our planner still steps QUAD4 out; the state shows
  Take-off until the hand-over, so his own take-off phase (0.4 s in sq8_2) has no step of its own.
- `.claude/skills/thesis-figure/scripts/figure_audit.py` warns that the fonts are not Computer Modern (intended:
  IEEE) and flags "Offset"/"plate" in the attach figure's two-line label as overlapping: their word boxes touch, the
  ink does not (checked at 600 dpi).
