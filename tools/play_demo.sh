#!/bin/bash
# Replay a recorded Gazebo run in the GUI at real speed and optionally film it.
#   tools/play_demo.sh R0736            # run id, or a run directory
# Filming: press play, click the grey camera button to start, click it again to stop, then CLOSE THE WINDOW
# without using the save dialog (it crashes on this machine). The clip is moved to ~/Videos/<run>_demo_<n>.mp4.
# Never run while the rig stack is up.
REPO=/home/wesley/multi_drone_control
arg="$1"
if [ -d "$arg" ]; then dir="$arg"; else dir=$(ls -d "$REPO"/results/*/"${arg}"_* 2>/dev/null | head -1); fi
[ -n "$dir" ] && [ -f "$dir/gz_record/state.tlog" ] || { echo "no recording for '$arg' (expected <run>/gz_record/state.tlog)"; exit 1; }
export GZ_SIM_RESOURCE_PATH="$REPO/simulation_assets:$REPO/simulation_assets/models:${GZ_SIM_RESOURCE_PATH}"
work=$(mktemp -d); cd "$work"      # the recorder writes ign_recording.mp4 into the working directory
echo "Replaying $dir"
gz sim --playback "$dir/gz_record" --gui-config "$REPO/tools/demo_playback_gui.config"
mkdir -p "$HOME/Videos"; run=$(basename "$dir" | cut -d_ -f1); n=1
for f in "$work"/*.mp4; do
  [ -f "$f" ] || continue
  while [ -e "$HOME/Videos/${run}_demo_${n}.mp4" ]; do n=$((n+1)); done
  mv "$f" "$HOME/Videos/${run}_demo_${n}.mp4" && echo "saved ~/Videos/${run}_demo_${n}.mp4"
done
rmdir "$work" 2>/dev/null || true
