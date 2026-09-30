#!/usr/bin/env bash
# Run one command in a fresh mdc-humble container on the lab PC.
#   tools/remote/lab_run.sh [--tag T] [--cpus N] [--domain ID] [--partition P]
#                           [--env K=V]... [--mount HOSTDIR:CTRDIR]... [--workdir D] -- <command...>
# The command runs in /home/wesley/multi_drone_control (drones:~/mdc bind-mounted) with ROS
# and the workspace sourced. Each container gets its own bridge network, so DDS and
# gz-transport discovery cannot reach other runs.
set -euo pipefail
LAB="${LAB_HOST:-drones@drones}"
DEST="${LAB_DIR:-mdc}"
IMAGE="${LAB_IMAGE:-mdc-humble:db1c3ca}"
TAG="run$$"; CPUS=4; DOMAIN=40; PARTITION=""; EXTRA=""; MKDIRS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2 ;;
    --cpus) CPUS="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --partition) PARTITION="$2"; shift 2 ;;
    --env) EXTRA+=" -e $(printf '%q' "$2")"; shift 2 ;;
    # HOSTDIR (no spaces) relative to the lab user's home unless absolute; created if missing
    --mount) H="${2%%:*}"; [ "${H#/}" != "$H" ] || H="\$HOME/$H"
             EXTRA+=" -v $H:${2#*:}"; MKDIRS+=" $H"; shift 2 ;;
    --workdir) EXTRA+=" -w $(printf '%q' "$2")"; shift 2 ;;
    --) shift; break ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[ $# -gt 0 ] || { echo "usage: $0 [--tag T] [--cpus N] [--domain ID] [--partition P] -- cmd..." >&2; exit 2; }
PARTITION="${PARTITION:-wesley_$TAG}"
NAME="wesley_$TAG"
CMD="$(printf '%q ' "$@")"

REMOTE=$(cat <<R
set -e
${MKDIRS:+mkdir -p $MKDIRS}
docker network create "$NAME" >/dev/null
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true; docker network rm "$NAME" >/dev/null 2>&1 || true' EXIT
docker run --rm --init --name "$NAME" --network "$NAME" --cpus "$CPUS" --shm-size 1g \
  -v "\$HOME/$DEST:/home/wesley/multi_drone_control" \
  -e ROS_DOMAIN_ID="$DOMAIN" -e GZ_PARTITION="$PARTITION"$EXTRA \
  "$IMAGE" bash -c $(printf '%q' "$CMD")
R
)
if [ "$(hostname)" = "drones" ]; then
  bash -c "$REMOTE"
else
  ssh -o BatchMode=yes "$LAB" "bash -c $(printf '%q' "$REMOTE")"
fi
