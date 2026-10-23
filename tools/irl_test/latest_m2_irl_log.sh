#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../.." && pwd)"
latest=""

shopt -s nullglob
for candidate in "$repo_root"/logs/m2_irl/mission_*; do
    [[ -d "$candidate" ]] || continue
    if [[ -z "$latest" || "$candidate" > "$latest" ]]; then
        latest="$candidate"
    fi
done

if [[ -z "$latest" ]]; then
    printf 'No M2 IRL mission logs found under %s\n' "$repo_root/logs/m2_irl" >&2
    exit 1
fi
printf '%s\n' "$latest"
