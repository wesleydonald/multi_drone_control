#!/usr/bin/env bash
# backup_results.sh — copy flight data off this machine.
#
# A lab session is not finished until this has run. See general/THESIS_PLAN.md §4.
#
#   ./tools/backup_results.sh                 # back up to every configured destination
#   ./tools/backup_results.sh --dry-run       # show what would transfer
#   ./tools/backup_results.sh --verify        # restore drill: prove a backup is readable
#   ./tools/backup_results.sh /media/wesley/EXT   # one-off destination
#
# Destinations come from MDC_BACKUP_DEST (colon-separated, like PATH) or the
# command line. Put the export in ~/.bashrc so it is never forgotten:
#
#   export MDC_BACKUP_DEST="/media/wesley/THESIS_BACKUP:/home/wesley/OneDrive/thesis_results"
#
# rsync is used with --checksum rather than the default size+mtime comparison:
# a CSV that is rewritten with the same length by a re-run would otherwise be
# silently skipped.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIRS=("results" "results_archive")
STAMP="$(date +%Y-%m-%dT%H:%M:%S)"
LOGFILE="$REPO/results/.backup_history.log"

DRY_RUN=0
VERIFY=0
CLI_DESTS=()

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --verify)  VERIFY=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *)         CLI_DESTS+=("$arg") ;;
  esac
done

# ── Resolve destinations ────────────────────────────────────────────────────
DESTS=()
if [ ${#CLI_DESTS[@]} -gt 0 ]; then
  DESTS=("${CLI_DESTS[@]}")
elif [ -n "${MDC_BACKUP_DEST:-}" ]; then
  IFS=':' read -r -a DESTS <<< "$MDC_BACKUP_DEST"
fi

if [ ${#DESTS[@]} -eq 0 ]; then
  cat >&2 <<'EOF'
!! No backup destination configured.

   Set MDC_BACKUP_DEST (colon-separated, like PATH) or pass one as an argument:

     export MDC_BACKUP_DEST="/media/wesley/THESIS_BACKUP:/home/wesley/OneDrive/thesis"

   Refusing to exit 0 with nothing backed up -- a backup script that quietly
   does nothing is worse than no backup script.
EOF
  exit 2
fi

# ── Restore drill ───────────────────────────────────────────────────────────
# An untested backup is not a backup. Picks a random run from the first
# destination, copies it back to a scratch dir, and compares every checksum.
if [ "$VERIFY" -eq 1 ]; then
  DEST="${DESTS[0]}"
  echo "── restore drill from $DEST ──"
  RUN="$(find "$DEST/results" -mindepth 2 -maxdepth 2 -type d 2>/dev/null | shuf -n1 || true)"
  if [ -z "$RUN" ]; then
    echo "!! no runs found at $DEST/results -- has a backup ever completed?" >&2
    exit 3
  fi
  SCRATCH="$(mktemp -d)"
  trap 'rm -rf "$SCRATCH"' EXIT
  cp -a "$RUN" "$SCRATCH/"
  REL="${RUN#"$DEST"/}"
  if [ ! -d "$REPO/$REL" ]; then
    echo "   restored $REL ($(du -sh "$RUN" | cut -f1)); no local copy to compare against."
    echo "   readable: OK"
    exit 0
  fi
  if diff -r "$RUN" "$REPO/$REL" >/dev/null 2>&1; then
    echo "   $REL — restored and byte-identical to local. OK"
  else
    echo "!! $REL — restored copy DIFFERS from local. Investigate before trusting this backup." >&2
    exit 4
  fi
  exit 0
fi

# ── Backup ──────────────────────────────────────────────────────────────────
RSYNC_OPTS=(-a --checksum --partial --human-readable --stats)
[ "$DRY_RUN" -eq 1 ] && RSYNC_OPTS+=(--dry-run)

FAILED=0
for DEST in "${DESTS[@]}"; do
  echo "── backing up to $DEST ──"
  if [ ! -d "$DEST" ]; then
    echo "!! destination missing: $DEST (drive not mounted?) — SKIPPED" >&2
    FAILED=1
    continue
  fi
  # A copy on the same physical device survives an accidental rm -rf, but NOT a
  # disk failure. Say which kind of protection this actually is, so a same-disk
  # snapshot is never mistaken for an off-machine backup.
  if [ "$(stat -c %d "$REPO")" = "$(stat -c %d "$DEST")" ]; then
    echo "   NOTE: same filesystem as the source — protects against accidental"
    echo "         deletion only, NOT against disk failure, loss or theft."
  fi
  for SRC in "${SRC_DIRS[@]}"; do
    [ -d "$REPO/$SRC" ] || continue
    mkdir -p "$DEST/$SRC"
    rsync "${RSYNC_OPTS[@]}" "$REPO/$SRC/" "$DEST/$SRC/" | tail -n 12
  done
  if [ "$DRY_RUN" -eq 0 ]; then
    N_LOCAL=$(find "$REPO/results" -type f 2>/dev/null | wc -l)
    N_DEST=$(find "$DEST/results" -type f 2>/dev/null | wc -l)
    echo "   files local=$N_LOCAL dest=$N_DEST"
    [ "$N_DEST" -lt "$N_LOCAL" ] && { echo "!! destination has FEWER files than source" >&2; FAILED=1; }
    echo "$STAMP  $DEST  local=$N_LOCAL dest=$N_DEST" >> "$LOGFILE"
  fi
done

if [ "$FAILED" -ne 0 ]; then
  echo; echo "!! one or more destinations failed — data is NOT fully backed up." >&2
  exit 1
fi
[ "$DRY_RUN" -eq 0 ] && echo && echo "   backup complete ($STAMP)"
