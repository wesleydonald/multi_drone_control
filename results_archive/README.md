# results_archive/

**Tracked by git and pushed to GitHub.** This is the only copy of your flight data
that exists off this laptop.

## The deal (decided 2026-08-04)

| Tree | Tracked? | Off-machine copy? | Holds |
|---|---|---|---|
| `results/` | no | **no** | every raw run, sim and hardware |
| `results_archive/` | **yes** | **yes, via GitHub** | the curated runs the thesis depends on |

Raw runs live on one disk only. There is also a convenience snapshot at
`~/thesis_backups/multi_drone_control` (outside the repo, so a `git clean -xdf` or
an `rm -rf` of a build directory cannot reach it) — refresh it with
`./tools/backup_results.sh ~/thesis_backups/multi_drone_control`. It is on the same
physical disk, so it protects against deletion, not against drive failure.

**The practical consequence:** the moment a run produces a number, a figure, or a
claim you intend to defend, promote it. Once promoted and pushed it is safe;
until then it is not.

```bash
./tools/archive_run.sh results/2026-10-14/R0142_real_attach_circle "Fig 7.3 mid-flight attach"
git add results_archive && git commit -m "Archive R0142 (Fig 7.3)" && git push
```

## What belongs here

- Every run behind a thesis figure or table.
- The baseline runs the gate thresholds were derived from.
- Runs that demonstrate a failure worth showing (the attach runaway, the elevation
  boundary endpoints) — negative results carry the boundary claim (N4).
- Hardware sessions in full. They are the most expensive and least repeatable data
  you have; a lab day cannot be re-run from a laptop.

## What does not

- Exploratory sim sweeps. Keep the `metrics.json` summary if the aggregate matters,
  not 500 raw runs.
- Anything reproducible in minutes from a config that is already tracked.

## Rules

1. **Never rewrite a file in here.** If an analysis changes, archive a new run and
   supersede the old one in `ARCHIVED.json`. Thesis figures must stay traceable to
   the exact bytes they were made from.
2. `tools/thesis_figures.py` regenerates every figure from this directory alone. If
   a figure cannot be rebuilt from what is here, something is missing.
3. Push after every archive. An archive that is only local is not a backup.
