#!/usr/bin/env bash
# DEPRECATED — moved to tools/clean_slate.sh (2026-08-04).
#
# The previous contents of this file had a duplicated block pasted into it and
# failed `bash -n` at line 32, so it could never run. Anything that "worked"
# after calling it did so by luck. The replacement also clears Fast-DDS shared
# memory and exits non-zero when the machine is not clean.
#
# This shim forwards to the new script so existing habits and notes keep working.
exec "$(dirname "${BASH_SOURCE[0]}")/tools/clean_slate.sh" "$@"
