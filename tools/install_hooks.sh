#!/usr/bin/env bash
# Point git at the repo's tracked hooks (THESIS_PLAN §9.5).
#
#     ./tools/install_hooks.sh          # enable
#     ./tools/install_hooks.sh --off    # back to .git/hooks
#
# core.hooksPath rather than copying into .git/hooks: a copy goes stale the moment the
# tracked hook changes, and nothing tells you.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

if [ "${1:-}" = "--off" ]; then
  git config --unset core.hooksPath || true
  echo "hooks disabled (git config core.hooksPath unset)"
  exit 0
fi

git config core.hooksPath tools/hooks
chmod +x tools/hooks/* 2>/dev/null || true

# Verify it took. `git config` succeeding says the value was written, not that git will
# use it -- a stale .git/hooks/pre-push or a per-worktree override both quietly win.
ACTUAL="$(git config --get core.hooksPath || true)"
if [ "$ACTUAL" != "tools/hooks" ]; then
  echo "!! core.hooksPath is '$ACTUAL', not 'tools/hooks' — hooks are NOT active"
  exit 1
fi
echo "hooks enabled: core.hooksPath = tools/hooks"
echo "  pre-push -> tools/gate.sh   (bypass with: git push --no-verify)"
