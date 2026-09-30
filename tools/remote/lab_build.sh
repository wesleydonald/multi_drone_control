#!/usr/bin/env bash
# Build the parity image mdc-humble:db1c3ca on the lab PC, then colcon-build ~/mdc in it.
#   tools/remote/lab_build.sh            image (if missing) + workspace
#   tools/remote/lab_build.sh --image    rebuild the image even if it exists
#   tools/remote/lab_build.sh --ws       workspace only
# Run from the laptop: it stages the build context (t_renderer, repacked gz debs) and syncs.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB="${LAB_HOST:-drones@drones}"
DEST="${LAB_DIR:-mdc}"
IMAGE="${LAB_IMAGE:-mdc-humble:db1c3ca}"
MODE="${1:-all}"
lab() { ssh -o BatchMode=yes "$LAB" "$@"; }

cp -u "$HOME/acados/bin/t_renderer" "$HERE/t_renderer"
"$HERE/repack_gz_debs.sh" | tail -1
"$HERE/lab_sync.sh"

if [ "$MODE" = "--image" ] || { [ "$MODE" = "all" ] && ! lab docker image inspect "$IMAGE" >/dev/null 2>&1; }; then
  # Detached on the lab side: a dropped ssh must not cancel a 30-minute build.
  # Host network: DNS does not resolve from the lab PC's docker bridge.
  lab "rm -f ~/mdc_image_build.log; cd ~/$DEST/tools/remote && nohup sh -c 'docker build --network host -t $IMAGE . ; echo BUILD_EXIT=\$?' > ~/mdc_image_build.log 2>&1 &"
  echo "image build started; log drones:~/mdc_image_build.log"
  until lab "grep -q '^BUILD_EXIT=' ~/mdc_image_build.log"; do sleep 120; done
  lab "grep -q '^BUILD_EXIT=0' ~/mdc_image_build.log" || { lab 'tail -30 ~/mdc_image_build.log'; exit 1; }
  lab docker image ls "$IMAGE"
fi

[ "$MODE" = "--image" ] && exit 0
"$HERE/lab_run.sh" --tag colcon --cpus 12 -- colcon build --symlink-install --parallel-workers 12
