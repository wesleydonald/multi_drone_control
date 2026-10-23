# Morning report, Sun 18 Oct 2026 (overnight 17-18 Oct, laptop, flight code 9d30665)

67 runs, R2174-R2240 (21:04-06:30). Registry rows are appended, with corrections R2192c/c2, R2193c, R2194c, R2199c,
R2200c/c2, R2172c and R2173c. Everything is on `day-1016`, nothing pushed. The thesis rewrite brief is ready for a
separate session (`docs/thesis_rewrite_brief_2026-10-17.md`).

## Rig findings first

| What | Runs | Result | Verdict |
|---|---|---|---|
| **0.55 m bands for the rig day.** Each carry cell gets the twin range of path error and ring tilt that a rig flight should land in. | R1847-R1876 + R2201-R2215 (3 per cell) | FROZEN ~06:17. Upper bounds: 26-48 mm path error, 4.5-9.3 deg tilt by cell. All 15 runs landed with no faults. The laptop's third run is the highest of the three in every cell (two hosts in the band). | frozen by the signed rule [no reviewer: the rule is the claim] |
| **The touchdown fix (A) with QUAD4 welded and folded in.** This was the critic's must-fix 1 before A goes on the rig. | R2220-R2223, R2228, R2233, R2234 | In all 7 landings the stall test fired with QUAD4 in the list, never the max-drop fallback. The stamp fix works. | met [critic's re-fly; reviewer not needed for a must-fix] |
| **The height integral winds up under a radius error.** With a +2 cm radius error the vertical integral climbs while the ring stays 0.8 cm high. | R2216-R2219 | Measured pull (the default): the integral reaches its bound of 2.14 N about 99 s after lift and stays there. Ring height and landing do not change. Planned pull: the integral stays near 0 and the height error is within 0.6 cm, but the ring tilts 1.8 deg. | the measured pull absorbs the integral: wind-up, harmless in the twin [exploratory] |
| **A false attach in the twin.** The twin's no-catch scenario is meant to reject every contact. | R2228 | The proof passed on try 3 with no catch (tip still on the plate, tether taut). The twin skips the plate-motion check; the rig rule keeps it plus the taut-length check. | not a rig failure; the twin cannot test this case [exploratory] |

## Twin

| What | Runs | Result | Verdict |
|---|---|---|---|
| **No-floor LAND (rev 3).** After the ring lands, the reference keeps descending until the stall test fires, so a drone stronger than its thrust map lands instead of hanging. | R2174-R2200 (17 switch, 10 base) | F1: the residual is gone in 6/6 (base: the strong drone hangs 15.8-22.3 cm up and tips 28-90 deg). F2, F4 and F5 pass. F3 fails as written once: R2199's drone 0 reads 7.1 deg on the floor against 6.5. Contact speed is 0.17-0.31 m/s against 0.01-0.12 in the bases, under the 0.40 bar. | falsifier fires; stays off [not supported: F3 by 0.6 deg; the QUAD4 arm flew without A] |
| **The legacy profile has no A.** Old-world QUAD4 configs fly `legacy:=true`, and `sim_legacy.yaml` does not set the touchdown fix. | R2172, R2173, R2192, R2193, R2200, R2237-R2240 | A was off in all of them (planner read-back). Every one landed. | a profile gap |
| **Orbit joins.** Tejen approaches the circling ring. | R2229-R2232 | 3 of 4 dropped the ring (tilt over 30 deg) during his approach; one held at 20 deg. All three rod tensions rose 3.1 to 4.7 N at even throttle, with the planner ticking normally. Cause open. The same configs passed on 12 Oct. | twin only (the rig refuses an orbit under partner_contact) [exploratory] |
| **Hover joins.** | R2220-R2228, R2233-R2236 | 11 of 13 land. In R2235 and R2236 (−x push) QUAD4 tips at LAND: TB14, the rigid-rod artefact, 2 s after ring touchdown and before A can act. Tejen's rod passed within 0.000-0.063 m of a carrier rod in 3 runs. `m1c_hover_2` did not fly (its config file is missing). | not thesis evidence [exploratory] |

## Decisions for you

1. **A on the rig.** Recommended: do the two should-fixes first (log the count of stale-pose resets; read one rig log
   through a Motive dropout), then turn A on through a `configs/rig/` params file. Alternatively, leave A off on the
   rig for 21-22 Oct.
2. **The no-floor switch.** Recommended: re-card F3 per drone against the switch-off base, since drone 0 has no A-only
   floor reading, and fly 3 rig-path pairs. The rig's measured kT spread (33.75-36.89 against 35.2) means the
   strong-map case is real. Alternatively, park it.
3. **A in the legacy profile.** Recommended: add `land_rearm_touchdown: true` to `sim_legacy.yaml`, a launch default
   that needs your word, so every twin QUAD4 config flies A.
4. **The orbit-join ring drops.** Recommended: look after the rig day, since an orbit is refused on the rig.
   Alternatively, now.
5. **Merge `day-1016`** when you are ready.
