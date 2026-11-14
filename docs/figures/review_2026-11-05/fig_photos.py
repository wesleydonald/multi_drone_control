#!/usr/bin/env python3
"""Photo figures of Setup 3 in flight (Wesley's action shots of 12 Nov, RIG-1112-fm9_5 and fm9hc_2; sent 13 Nov:
"replace the current stuff that is just animated or drawn or old versions of the setup ... annotate these as you like,
do not use the full storyboard photos").

    fig_intro_photos    Figure 1: (a) overhead, four quadrotors carrying the ring with the object in its net (frame 06; the new agent is the
                        quadrotor left of the ring, back on plate 9),
                        (b) corner camera, the new agent dropping the object into the net (frame 15).
    fig_mission_photos  4.2, the full mission from above (frames 02, 03, 04, 05 of fm9_5): (a) the new agent detached
                        and flying to the object, (b) bringing it to the ring, (c) dropping it into the net,
                        (d) coming down to reattach with the object in the net. Words only in (a).
    fig_reattach_photo  4.2.3, the new agent coming back down to plate 9 (corner camera frame 16, fm9_5).
    fig_drop_photos     4.2, (a) the object lifted on the new agent's cable (ground camera, frame 09), (b) the circling
                        ring, a 24 s trail from above (frame 10, fm9hc_2).

Each overhead frame of the sequence uses the same crop, so the panels share one scale. The photos are downsampled to
about 330 dpi at print size. Source folder: PHOTOS (the zip's "Action Shots"), outside the repo.
"""
import glob
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402

S.HERE = HERE
PHOTOS = os.environ.get('MDC_PHOTOS', '/tmp/claude-1000/-home-wesley-multi-drone-control/'
                        'a075d8b7-a0e1-498e-b597-9702885094af/scratchpad/action_shots_zip/Action Shots')
BOX = dict(boxstyle='round,pad=0.18', fc='white', ec='none', alpha=0.85)


def photo(num, box, width_px=1000):
    """Frame `num` cropped to box = (x0, y0, x1, y1) in its own pixels, resized to width_px. 's1/06' and 's2/08' are
    frames of the run stories (s1: fm9_5, the full mission, corner camera 4K; s2: fm9hc_4, the drop into the circling
    ring and the reattachment, corner camera)."""
    if num.startswith('s'):
        story, n = num[1:].split('/')
        (f,) = glob.glob(os.path.join(PHOTOS, 'Run stories', f'{story} - *', f'{n} *.jpg'))
    else:
        (f,) = glob.glob(os.path.join(PHOTOS, f'{num} *.jpg'))
    im = Image.open(f).convert('RGB').crop(box)
    h = round(im.size[1] * width_px / im.size[0])
    return np.asarray(im.resize((width_px, h), Image.LANCZOS)), box, width_px / (box[2] - box[0])


def show(ax, img, letter, text=None):
    """The photo, and '(a) Its title' under it (Wesley 14 Nov: titles by the letters, first word capitalised)."""
    ax.imshow(img[0], interpolation='none')
    ax.set_axis_off()
    ax.text(0.5, -0.03, f'({letter})' + (rf'\ \ {text}' if text else ''), transform=ax.transAxes, ha='center', va='top',
            fontsize=9)


def label(ax, img, text, xy, xytext):
    """Text at xytext pointing to xy, both in the frame's own pixels."""
    (x0, y0, _, _), s = img[1], img[2]
    p = ((xy[0] - x0) * s, (xy[1] - y0) * s)
    q = ((xytext[0] - x0) * s, (xytext[1] - y0) * s)
    ax.annotate(text, xy=p, xytext=q, fontsize=8, ha='center', va='center', bbox=BOX,
                arrowprops=dict(arrowstyle='-', lw=0.7, color='white', shrinkA=2, shrinkB=3))


def intro():
    # Wesley 14 Nov (2nd): (a) the new agent reattaching (fm9hc_4 story frame 09, was (b)), (b) the four quadrotors steady
    # after the reattachment with the object in the net (fm9_5 story frame 09); both corner camera; the object not labelled;
    # the new agent labelled in both (in (b) it is the second quadrotor from the left, Wesley 14 Nov)
    a = photo('s2/09', (1000, 300, 2150, 1180), 900)
    b = photo('s1/09', (2050, 380, 2950, 1120), 900)
    fig = S.figure('full', 2.3)
    axs = fig.subplots(1, 2, gridspec_kw=dict(width_ratios=[1150 / 880, 900 / 740]))   # equal heights
    show(axs[0], a, 'a', 'The new agent reattaching')
    show(axs[1], b, 'b', 'Four quadrotors carrying the payload')
    label(axs[0], a, 'New agent', (1462, 400), (1250, 340))
    label(axs[1], b, 'New agent', (2348, 493), (2560, 430))   # Wesley 14 Nov: second from the left
    S.save(fig, 'fig_intro_photos')


def mission():
    box = (0, 40, 1650, 960)
    frames = [photo(n, box, 1000) for n in ('02', '03', '04', '05')]
    fig = S.figure('full', 3.85)
    axs = fig.subplots(2, 2)
    names = ('Flying to the object', 'Bringing the object', 'Dropping it into the net', 'Coming down to reattach')
    for ax, img, letter, name in zip(axs.flat, frames, 'abcd', names):
        show(ax, img, letter, name)
    label(axs[0, 0], frames[0], 'New agent', (567, 470), (560, 260))
    label(axs[0, 0], frames[0], 'Object', (553, 680), (560, 850))
    S.save(fig, 'fig_mission_photos')


def drop():
    # Wesley 14 Nov: both from the corner; the overhead trail (frame 10) was hard to read
    a = photo('s1/06', (1100, 1000, 2500, 2100), 900)
    b = photo('s2/05', (950, 60, 2150, 1170), 900)
    fig = S.figure('full', 2.55)
    axs = fig.subplots(1, 2, gridspec_kw=dict(width_ratios=[1400 / 1100, 1200 / 1110]))
    show(axs[0], a, 'a', 'Magnet on the object')
    show(axs[1], b, 'b', 'Object over the circling payload')
    label(axs[0], a, 'New agent', (1645, 1284), (1900, 1180))
    label(axs[0], a, 'Object', (1688, 1905), (1950, 1850))
    label(axs[1], b, 'New agent', (1481, 150), (1150, 160))
    label(axs[1], b, 'Object', (1425, 797), (1150, 760))
    S.save(fig, 'fig_drop_photos')


def reattach():
    # Wesley 14 Nov: (a) the new agent just as it is about to attach (fm9hc_4, corner), (b) frame 16 as before
    a = photo('s2/08', (1120, 180, 1840, 1150), 700)
    b = photo('16', (430, 110, 1370, 830), 900)
    fig = S.figure(4.95, 2.3)                        # fits under tab:res_reattach; built at its own width, no gap
    axs = fig.subplots(1, 2, gridspec_kw=dict(width_ratios=[720 / 970, 940 / 720]))
    show(axs[0], a, 'a', 'Just before attaching')
    show(axs[1], b, 'b', 'Coming down to plate 9')
    label(axs[0], a, 'New agent', (1500, 272), (1660, 520))
    label(axs[1], b, 'New agent', (675, 245), (900, 330))   # second from the left, as in fm9_5's frame 09 (Wesley 14 Nov)
    S.save(fig, 'fig_reattach_photo', width=4.95)

if __name__ == '__main__':
    S.setup()
    for name in sys.argv[1:] or ('intro', 'mission', 'drop', 'reattach'):
        globals()[name]()
