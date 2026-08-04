#!/usr/bin/env bash
# archive_run.sh — promote a run from results/ into results_archive/.
#
# results_archive/ is tracked and pushed, so this is what puts a run somewhere
# other than this one laptop. Raw results/ is single-copy by decision
# (2026-08-04), which makes promotion the moment data becomes safe.
#
#   ./tools/archive_run.sh results/2026-10-14/R0142_real_attach_circle "Fig 7.3 attach"
#   ./tools/archive_run.sh --list
#
# The reason string is mandatory and is recorded in ARCHIVED.json. In four months
# you will not remember why a given run was kept, and an archive you cannot
# interpret is not much better than no archive.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCHIVE="$REPO/results_archive"

if [ "${1:-}" = "--list" ]; then
  echo "── archived runs ──"
  find "$ARCHIVE" -name ARCHIVED.json -print0 2>/dev/null | while IFS= read -r -d '' f; do
    d="$(dirname "$f")"
    printf "  %-45s %s\n" "${d#"$ARCHIVE"/}" \
      "$(python3 -c "import json,sys;print(json.load(open('$f')).get('reason',''))" 2>/dev/null)"
  done
  exit 0
fi

SRC="${1:-}"
REASON="${2:-}"

if [ -z "$SRC" ] || [ -z "$REASON" ]; then
  sed -n '2,14p' "$0"
  exit 2
fi
[ -d "$SRC" ] || { echo "!! not a directory: $SRC" >&2; exit 2; }

NAME="$(basename "$SRC")"
DEST="$ARCHIVE/$NAME"

# Never rewrite an archived run: thesis figures must stay traceable to the exact
# bytes they were generated from.
if [ -e "$DEST" ]; then
  echo "!! already archived: $DEST" >&2
  echo "   Archived runs are immutable. To supersede it, archive the new run under" >&2
  echo "   its own run_id and note the supersession in its reason string." >&2
  exit 3
fi

mkdir -p "$ARCHIVE"
cp -a "$SRC" "$DEST"

GIT_SHA="$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
DIRTY="$(git -C "$REPO" status --porcelain 2>/dev/null | wc -l)"
python3 - "$DEST" "$SRC" "$REASON" "$GIT_SHA" "$DIRTY" <<'PY'
import json, sys, os, datetime, hashlib
dest, src, reason, sha, dirty = sys.argv[1:6]
# Checksum the payload so later corruption or accidental edits are detectable.
h = hashlib.sha256()
files = []
for root, _, names in os.walk(dest):
    for n in sorted(names):
        if n == 'ARCHIVED.json':
            continue
        p = os.path.join(root, n)
        rel = os.path.relpath(p, dest)
        with open(p, 'rb') as f:
            data = f.read()
        h.update(rel.encode()); h.update(data)
        files.append(rel)
meta = {
    'archived_utc': datetime.datetime.utcnow().isoformat() + 'Z',
    'source_path': src,
    'reason': reason,
    'git_sha': sha,
    'working_tree_dirty_files': int(dirty),
    'file_count': len(files),
    'sha256_of_contents': h.hexdigest(),
}
with open(os.path.join(dest, 'ARCHIVED.json'), 'w') as f:
    json.dump(meta, f, indent=2, sort_keys=True)
print(f"   archived {len(files)} file(s), sha256 {meta['sha256_of_contents'][:16]}...")
if int(dirty):
    print(f"   NOTE: working tree had {dirty} uncommitted change(s) when this ran —")
    print( "         the recorded git SHA does not fully describe the code that produced it.")
PY

echo "   -> results_archive/$NAME"
echo
echo "   Now push it, or it is still only on this laptop:"
echo "     git add results_archive && git commit -m 'Archive $NAME' && git push"
