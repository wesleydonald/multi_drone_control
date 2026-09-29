#!/bin/bash
# Film a recorded Gazebo run to ~/Videos/<run>_<name>.mp4 with no clicks: replays the gz_record log in the GUI,
# points the camera, presses play over gz transport and films the window with GStreamer (x264, real speed).
#   tools/film_demo.sh R0736 [seconds_extra] [cam "x y z" look "x y z"]
# The window appears on screen while filming. Never run while the rig stack or another Gazebo is up.
set -u
REPO=/home/wesley/multi_drone_control
run="$1"; extra="${2:-5}"
cam="${3:-3.2 -3.2 2.4}"; look="${4:-0 0.3 0.5}"
dir=$(ls -d "$REPO"/results/*/"${run}"_* 2>/dev/null | head -1)
[ -n "$dir" ] && [ -f "$dir/gz_record/state.tlog" ] || { echo "no recording for $run"; exit 1; }
name=$(basename "$dir" | sed -E 's/^R[0-9]+_sim_gz_//')
out="$HOME/Videos/${run}_${name}.mp4"; mkdir -p "$HOME/Videos"
dur=$(python3 -c "import csv;r=list(csv.DictReader(open('$dir/logs/run.csv')));print(int(float(r[-1]['t']))+1)" 2>/dev/null || echo 150)
export GZ_PARTITION="film_$$"
export GZ_SIM_RESOURCE_PATH="$REPO/simulation_assets:$REPO/simulation_assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
cd "$REPO/simulation_assets"
cfg=$(mktemp --suffix=.config); cp "$REPO/tools/film_gui.config.clean" "$cfg"   # gz rewrites its GUI config on exit
gz sim --playback "$dir/gz_record" --gui-config "$cfg" > /tmp/film_gz_$$.log 2>&1 &
gzpid=$!
cleanup(){ kill -INT "${gstpid:-0}" 2>/dev/null; sleep 3; pkill -TERM -P $gzpid 2>/dev/null; kill -TERM $gzpid 2>/dev/null; }
trap cleanup EXIT
wid=""
for i in $(seq 1 90); do
  wid=$(xwininfo -root -tree 2>/dev/null | grep -i '"Gazebo Sim"' | grep -oE '0x[0-9a-f]+' | head -1)
  [ -n "$wid" ] && break; sleep 1
done
[ -n "$wid" ] || { echo "Gazebo window not found"; exit 1; }
sleep 8   # let the scene load
# camera: position and a quaternion looking from cam to look
q=$(python3 - "$cam" "$look" <<'PY'
import sys, math
c=[float(v) for v in sys.argv[1].split()]; l=[float(v) for v in sys.argv[2].split()]
dx,dy,dz=[l[i]-c[i] for i in range(3)]
yaw=math.atan2(dy,dx); pitch=-math.atan2(dz,math.hypot(dx,dy))
cy,sy,cp,sp=math.cos(yaw/2),math.sin(yaw/2),math.cos(pitch/2),math.sin(pitch/2)
print(f"{cy*cp} {-sy*sp} {cy*sp} {sy*cp}")   # w x y z for yaw then pitch (roll 0)
PY
)
read qw qx qy qz <<< "$q"; read cx cy cz <<< "$cam"
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 \
  --req "pose: {position: {x: $cx, y: $cy, z: $cz}, orientation: {w: $qw, x: $qx, y: $qy, z: $qz}}" >/dev/null 2>&1
sleep 2
geo=$(xwininfo -id $wid | awk '/Absolute upper-left X/{x=$4}/Absolute upper-left Y/{y=$4}/Width:/{w=$2}/Height:/{h=$2}END{print x,y,w,h}')
read gx gy gw gh <<< "$geo"; gw=$((gw/2*2)); gh=$((gh/2*2))
gst-launch-1.0 -e ximagesrc startx=$gx starty=$gy endx=$((gx+gw-1)) endy=$((gy+gh-1)) use-damage=false ! video/x-raw,framerate=30/1 ! videoconvert ! \
  video/x-raw,format=I420 ! x264enc speed-preset=veryfast bitrate=8000 key-int-max=60 ! mp4mux ! filesink location="$out" > /tmp/film_gst_$$.log 2>&1 &
gstpid=$!
sleep 1
gz service -s /world/default/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >/dev/null 2>&1 || echo "WARN: play request failed"
sleep 3
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 \
  --req "pose: {position: {x: $cx, y: $cy, z: $cz}, orientation: {w: $qw, x: $qx, y: $qy, z: $qz}}" >/dev/null 2>&1
echo "filming $run until the replay clock reaches ${dur} s -> $out"
start=$(date +%s)
while :; do
  simt=$(timeout 5 gz topic -e -n 1 -t /world/default/stats 2>/dev/null | awk '/^sim_time/{f=1} f&&/sec:/&&!/nsec/{print $2; exit}')
  now=$(date +%s)
  [ -n "$simt" ] && [ "$simt" -ge "$dur" ] && break
  [ $((now-start)) -gt $((dur*4+60)) ] && { echo "WARN: replay did not reach the end in time (at ${simt:-?} s)"; break; }
  sleep 2
done
sleep "$extra"
kill -INT $gstpid; wait $gstpid 2>/dev/null
echo "saved $out ($(du -h "$out" | cut -f1))"
python3 - "$out" <<'PY'
import cv2, sys
c=cv2.VideoCapture(sys.argv[1]); n=int(c.get(7)); dark=0; k=0
for fr in range(0, max(n,1), max(n//20,1)):
    c.set(1, fr); ok, im = c.read()
    if ok:
        k += 1; dark += im.mean() < 15
print(f"check: {k} sampled frames, {dark} dark" + ("  <-- WARNING: screen blanked or window hidden" if dark else ""))
PY
