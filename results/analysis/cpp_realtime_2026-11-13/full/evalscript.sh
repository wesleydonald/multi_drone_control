#!/usr/bin/env bash
# evalscript.sh <rig script>: the script with its final rig_flight call replaced by an offline evaluation of the same
# arrays (deskeval.py: LaunchContext walk, nothing started, no ROS graph). Prints the read-back and QUAD4's planned heights.
S="$1"
TMP=$(mktemp --suffix=.sh)
D=$(dirname "$(readlink -f "$0")")
sed -E 's#^tools/rig_flight.sh (real_control_launch.py) (.*) "\$\{1:-[A-Za-z0-9_]+\}"$#python3 '"$D"'/deskeval.py \1 \2#' "$S" > "$TMP"
grep -q "deskeval" "$TMP" || { echo "evalscript: no rig_flight line replaced in $S"; rm -f "$TMP"; exit 3; }
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=79
bash "$TMP"; rc=$?
rm -f "$TMP"; exit $rc
