#!/usr/bin/env python3
"""
tools/make_reel.py -- an evening's twin videos back to back, each after a 2 s title card, at real speed.

    tools/make_reel.py --date 2026-10-21                         # every rendered run of that day
    tools/make_reel.py results/2026-10-21/R2241_sim_gz_b2_hover ... --out results/videos/my_reel.mp4
    tools/make_reel.py --date 2026-10-21 --prune                 # and delete each checked run's state log

A run joins the reel once tools/render_video.py has rendered it (<run>/video/render.json, the camera ok and every
expected frame written). The title card gives the run id, the config and the registry row's verdict. The run id
stays in the corner of its video.

--prune deletes a run's gz_record (the Gazebo state log, 30-100 MB) once its video passes that check: the video is
what is kept (Wesley 21 Oct). Never run it while a live run or the rig stack is up on the same machine.
"""
import argparse
import csv
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
W, H, FPS = 1280, 720, 30


def verdicts():
    """Run id -> the verdict column of its registry row (the last row wins: a correction row follows the run)."""
    out = {}
    with open(os.path.join(REPO, 'results', 'registry.csv'), newline='') as fh:
        for row in csv.DictReader(fh):
            if row.get('run') and row.get('verdict'):
                out[row['run']] = row['verdict']
    return out


def checked_video(run_dir, camera):
    """(camera, mp4 path) of a fully rendered camera, or (None, why)."""
    try:
        with open(os.path.join(run_dir, 'video', 'render.json')) as fh:
            cams = json.load(fh)['cameras']
    except (OSError, ValueError, KeyError):
        return None, 'not rendered'
    names = [camera] if camera else list(cams)
    for name in names:
        c = cams.get(name)
        if c and c.get('ok') and c['frames'] == c['expected'] and os.path.exists(c['file']):
            return name, c['file']
    return None, 'render incomplete (see video/render.json)'


def ffmpeg(*args):
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', *args], check=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*', help='run directories (default: --date)')
    ap.add_argument('--date', help='every results/<date>/R*_sim_gz_* run')
    ap.add_argument('--camera', help='camera name (default: the first fully rendered one)')
    ap.add_argument('--out', help='default results/videos/reel_<date or first run>.mp4')
    ap.add_argument('--prune', action='store_true', help="delete each checked run's gz_record")
    a = ap.parse_args()
    runs = list(a.runs) or sorted(glob.glob(os.path.join(REPO, 'results', a.date or '-', 'R*_sim_gz_*')))
    if not runs:
        sys.exit('no runs: give run directories or --date')
    reg = verdicts()
    tag = a.date or os.path.basename(runs[0].rstrip('/'))
    out = a.out or os.path.join(REPO, 'results', 'videos', f'reel_{tag}.mp4')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    parts, kept = [], []
    with tempfile.TemporaryDirectory() as tmp:
        for i, run in enumerate(runs):
            run = run.rstrip('/')
            m = re.match(r'(R\d{4})_sim_gz_(.+)', os.path.basename(run))
            if not m:
                print(f'skip {run}: not a twin run directory')
                continue
            rid, name = m.groups()
            cam, mp4 = checked_video(run, a.camera)
            if cam is None:
                print(f'skip {rid}: {mp4}')
                continue
            card = os.path.join(tmp, f'{i:03d}_card.txt')
            with open(card, 'w') as fh:
                fh.write(f'{rid}   {name}\n\n{reg.get(rid, "(no registry row yet)")[:90]}')
            title = os.path.join(tmp, f'{i:03d}_a.mp4')
            ffmpeg('-f', 'lavfi', '-i', f'color=c=black:s={W}x{H}:d=2:r={FPS}',
                   '-vf', f"drawtext=fontfile={FONT}:textfile={card}:fontcolor=white:fontsize=34:"
                          f"line_spacing=14:x=(w-text_w)/2:y=(h-text_h)/2",
                   '-c:v', 'libx264', '-pix_fmt', 'yuv420p', title)
            body = os.path.join(tmp, f'{i:03d}_b.mp4')
            ffmpeg('-i', mp4, '-vf', f'scale={W}:{H},fps={FPS},drawtext=fontfile={FONT}:text={rid}:'
                                     f'fontcolor=white:fontsize=24:x=16:y=16:box=1:boxcolor=black@0.5',
                   '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', body)
            parts += [title, body]
            kept.append((rid, run, cam))
            print(f'{rid} {name}: {cam}')
        if not parts:
            sys.exit('nothing rendered to join')
        lst = os.path.join(tmp, 'list.txt')
        with open(lst, 'w') as fh:
            fh.writelines(f"file '{p}'\n" for p in parts)
        ffmpeg('-f', 'concat', '-safe', '0', '-i', lst, '-c', 'copy', out)
    print(f'reel: {out} ({len(kept)} runs)')
    if a.prune:
        for rid, run, _ in kept:
            rec = os.path.join(run, 'gz_record')
            if os.path.isdir(rec):
                shutil.rmtree(rec)
                print(f'pruned {rid}: gz_record')


if __name__ == '__main__':
    main()
