# Internal RA-L review, 23 Oct draft

Verdict: major revision. A clear, useful problem and a well-designed twin study, but the abstract and contributions
claimed more than the data show (hardware scope, the catch-offset meaning, the 8->4->8 chain), and there was no
limitations paragraph.

| # | Weakness | Fix |
|---|----------|-----|
| 1 | Abstract and contributions implied hardware validation of rejoin, attach and the 8-carrier chain; only the release was flown on the rig. | Contribution 4 now says rig = release, twin = release/rejoin/attach/chain; abstract names the twin for those results. |
| 2 | "A newcomer's magnet can catch up to 16 cm off its plate" contradicts Sec. IV-C, which says past ~2.5 cm the sweep measures tolerance to model error, not a real catch. "6-8 cm" quoted 8 cm that is still pending. | Abstract, contribution 3 and discussion recast as the fold-in's tolerance to catch-point error, 16 cm radial / 6 cm along the rim; the 2.5 cm caveat repeated in the discussion. |
| 3 | The measured-vs-planned pull lever was called "planning from the measured cable pull", but in Sec. III-A it is the trackers' feed-forward. | Reworded in abstract, contribution 2 and Sec. IV-B to "feeding the trackers the measured cable pull"; abstract quantifies it (about two thirds at 1 m). |
| 4 | Inconsistent numbers: rejoin text said every fold-in tilted 1.3-3.4 deg, then reported a 5.28 deg run; conclusion repeated it. Sec. IV-B said the 0.55 m 165 deg arm "landed", while the table says held. Rig tilt range 6.5-17.8 was in the table but not the text. | "10 of the 11 fold-ins" (text, conclusion, abstract); "held (one run)"; range added to text. |
| 5 | System said links are 1 m "in the results here", but the rig release table uses 0.55 m rods. Discussion claimed twin and rig "agreed" despite different links and back-in-band 1.8 vs 3.4 s. | System states 0.55 m rig rods vs 1 m twin cables; agreement claim softened in Results and Discussion. |
| 6 | Missing definitions: "in band", "tilt rise" vs the drop line acting on absolute tilt; Method said "one SQP iteration per tick" while Sec. IV-D varies it. | Definitions added at the top of Results (band value is a placeholder); Method now says a fixed count, one unless stated. |
| 7 | Over-claims on scale: the 8->4->8 chain is one exploratory run at one iteration a tick, slower than real time; "two iterations converge up to four" is an offline replay of nine flights. | Chain labelled exploratory in the abstract, contributions, Results and conclusion; offline qualifier added; duplicate chain recap in Sec. IV-D cut to one sentence. |
| 8 | No limitations paragraph; small n (3 rig releases, 1-2 twin runs a cell, 0.55 m column n=1) and no baseline that does not resize. | Limitations paragraph added to the Discussion; n noted in Results and Sec. IV-B. |

## Left for Wesley
- The band definition: `\placeholder{band definition}` added in 04_results.tex; give its value.
- "planned arm stays 0.2-2.7 deg under the drop line" (1 m, 150 deg) did not match the table's rises (29.62/27.10 imply
  0.38/2.90 under 30 if the line acts on the rise). Removed the numbers; restore them if they are absolute tilt.
- Fig. rejoin caption: "re-welds 17.8 s after the rejoin command" lies outside the T row's 16.6-17.7 s + weld; check
  which run the figure shows.
- No baseline arm (e.g. survivors not resized) for the release: a reviewer will ask for it.
- Rig newcomer attach and the real-time N_max remain placeholders; the title and abstract lean on attach, which is
  twin-only until 29 Oct.
