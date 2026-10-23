#!/usr/bin/env bash
# rig_pin.sh -- after a T2 launch: keep display and recording off the performance cores and below the controllers.
#
#   tools/rig_pin.sh                    show what it would do (default)
#   tools/rig_pin.sh --apply            apply it (taskset + renice on exact PIDs; undone by relaunching)
#   tools/rig_pin.sh --apply --boost    also raise the control path to nice -10 (asks for sudo once)
#
# This laptop (Core 5 210H): CPUs 0-7 are the 4 performance cores (2 threads each), 8-11 the 4 efficiency cores.
# The control path (trackers, mocap, radios, mux, fleet manager, his MPC) stays free to use every core: on 14 Oct it
# alone wanted ~600 % of CPU, more than the performance cores hold. --boost gives it ~9x the CPU share of a nice-0
# process under contention (ordinary CFS priority, not real-time, so a runaway node cannot lock the machine).
# Display goes to the efficiency cores at the lowest priority; the bag recorder goes there at normal priority (its
# stalls blank every topic in the bag, RIG-1014-t5_2-corr). Run it again after every relaunch (new PIDs).
set -uo pipefail
E_CORES=8-11
apply=false; boost=false
for a in "$@"; do case "$a" in --apply) apply=true ;; --boost) boost=true ;; *) echo "rig_pin: unknown $a" >&2; exit 2 ;; esac; done

# program path suffix | cpu list | nice
rules=(
  "simulation_communication/lib/simulation_communication/fleet_viz|$E_CORES|19"
  "rviz2/lib/rviz2/rviz2|$E_CORES|19"
  "robot_state_publisher/lib/robot_state_publisher/robot_state_publisher|$E_CORES|19"
  "drone_magnet/lib/drone_magnet/attach_target_publisher|$E_CORES|19"
)
# the control path for --boost
control=(
  "tracker/lib/tracker/tracker"
  "drone_communication/lib/drone_communication/motion_capture_publisher_node"
  "drone_communication/lib/drone_communication/elrs_interface"
  "drone_magnet/lib/drone_magnet/elrs_mux"
  "drone_magnet/lib/drone_magnet/magnet_gate"
  "fleet_manager/lib/fleet_manager/fleet_manager"
  "tejen_mpc/lib/tejen_mpc/main"
)

n=0; boost_pids=()
while read -r pid exe rest; do
  prog="$exe"
  [[ "${exe##*/}" == python3* ]] && prog="${rest%% *}"    # a Python node runs as `python3 <its script> ...`
  for r in "${rules[@]}"; do
    IFS='|' read -r suffix cpus nice <<< "$r"
    [[ "$prog" == *"/$suffix" ]] || continue
    n=$((n + 1))
    echo "pid $pid ${prog##*/}: cpus $cpus, nice $nice"
    if $apply; then taskset -a -cp "$cpus" "$pid" >/dev/null && renice -n "$nice" -p "$pid" >/dev/null; fi
  done
  if [[ "${exe##*/}" == python3* && "$rest" == *"/ros2 bag record "* ]]; then
    n=$((n + 1))
    echo "pid $pid ros2 bag record: cpus $E_CORES, nice 0"
    if $apply; then taskset -a -cp "$E_CORES" "$pid" >/dev/null; fi
  fi
  if $boost; then
    for c in "${control[@]}"; do
      [[ "$prog" == *"/$c" ]] && boost_pids+=("$pid") && echo "pid $pid ${prog##*/}: nice -10 (boost)"
    done
  fi
done < <(ps -eo pid=,args=)

if $boost && $apply && [ "${#boost_pids[@]}" -gt 0 ]; then
  sudo renice -n -10 -p "${boost_pids[@]}" >/dev/null && echo "rig_pin: ${#boost_pids[@]} control processes at nice -10"
fi
[ "$n" -eq 0 ] && [ "${#boost_pids[@]}" -eq 0 ] && echo "rig_pin: nothing matched (is T2 up?)"
$apply || echo "(dry run: add --apply)"
