"""
Unit tests for tools/twin_video.py (a twin run rendered from its logs as a presentation video).

Built on a synthetic run whose geometry is known: a ring tilted 10 deg about x, three carriers on
the rim at 270/30/150 deg, QUAD4 hanging over its plate at +y with its tip on it. Each test names the
property it protects:

  * the rods end on the ring: a carrier's plate is ring pose + R(q) @ (r cos az, r sin az, attach_z),
    the drawn ring passes through it, and the drawn rod ends there
  * a window can be named by its events (CONTACT-5, CONTACT#2+1)
  * the event labels: WELD and MUX:ours at one instant are one label, the runner's 5 Hz GIVE_UP is
    one event, and the mux switch of a take-back is not also called a hand-over
  * a released carrier loses its rod; the column block after the carriers is QUAD4, its rod ends on
    its tip
  * a frame is 1920x1080, and a short window encodes to H.264 (when ffmpeg is there)
"""
import csv
import json
import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import twin_video as tv            # noqa: E402

R_RIM, Z_ATT, ROD, TILT = 0.225, 0.02, 0.55, 10.0
AZ = (270.0, 30.0, 150.0)
P0 = np.array([0.1, -0.2, 1.0])
EVENTS = [(0.5, 'ARM', ''), (1.0, 'TAKEOFF', ''), (2.0, 'LIFTED', '1.0'), (3.9, 'PHASE:LIFT_PROOF', ''),
          (4.0, 'CONTACT', ''), (5.0, 'RELEASE', '1'), (6.0, 'CONTACT', ''), (7.0, 'WELD', ''),
          (7.0, 'MUX:ours', ''), (8.0, 'GIVE_UP', ''), (8.2, 'GIVE_UP', ''), (9.0, 'LAND', ''),
          (9.8, 'LANDED', '')]


def rot_x(deg):
    a = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


def plate(az):
    a = np.radians(az)
    return P0 + rot_x(TILT) @ np.array([R_RIM * np.cos(a), R_RIM * np.sin(a), Z_ATT])


@pytest.fixture(scope='module')
def run_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp('R9999_sim_gz_synthetic')
    os.makedirs(d / 'logs')
    json.dump({'argv': ['tools/run_experiment.py', 'configs/experiments/synthetic.yaml'],
               'launch_args': {'num_drones': 3, 'attach_azimuths_deg': ','.join(f'{a:g}' for a in AZ),
                               'attach_radius': R_RIM, 'attach_z': Z_ATT, 'pivot_offset_z': -0.04,
                               'attach_x_offset': 0.0, 'attach_y_offset': R_RIM}},
              open(d / 'manifest.json', 'w'))
    q = [np.cos(np.radians(TILT) / 2), np.sin(np.radians(TILT) / 2), 0.0, 0.0]
    tip = plate(90.0)
    cols = ['t', 'payload_x', 'payload_y', 'payload_z', 'payload_qw', 'payload_qx', 'payload_qy', 'payload_qz',
            'payload_tilt_deg', 'tip_x', 'tip_y', 'tip_z', 'ball_x', 'ball_y', 'ball_z']
    for i in range(4):
        cols += [f'd{i}_x', f'd{i}_y', f'd{i}_z', f'd{i}_armed', f'd{i}_attached']
    with open(d / 'logs' / 'run.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for t in np.arange(0.0, 10.0 + 1e-9, 0.02):
            row = [t, *P0, *q, TILT, *tip, *P0]
            for i, az in enumerate(AZ):
                a = np.radians(az)
                out = np.array([np.cos(a), np.sin(a), 0.0])
                body = plate(az) + ROD * (0.5 * out + np.sqrt(0.75) * np.array([0, 0, 1])) + [0, 0, 0.04]
                row += [*body, 1, 0]
            row += [*(tip + [0, 0, 0.63]), 1, 0]
            w.writerow(row)
    with open(d / 'logs' / 'events.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_time', 'event', 'arg'])
        for e in EVENTS:
            w.writerow([f'{e[0]:.3f}', e[1], e[2]])
    return str(d)


def test_rods_end_on_the_ring(run_dir):
    S = tv.load(run_dir)
    assert (S['n_car'], S['n_all'], S['hi'], S['hi_name']) == (3, 4, 3, 'QUAD4')
    R = tv.quat_rot(np.array([[np.cos(np.radians(TILT) / 2), np.sin(np.radians(TILT) / 2), 0, 0]]))[0]
    pl = tv.plates(S, P0, R)
    ring = P0 + tv.ring_circle(S) @ R.T
    for i, az in enumerate(AZ):
        assert np.allclose(pl[i], plate(az))
        assert np.linalg.norm(ring - pl[i], axis=1).min() < 1e-9
    v = tv.Video(S, 0.0, 10.0, np.array([4.0]), 1.0, 'test', tv.VIEW)
    v.update(0)
    for i, az in enumerate(AZ):
        x, y = v.rods[i].get_data()
        ex, ey = v.cam.xy(plate(az))
        assert np.isclose(x[-1], ex) and np.isclose(y[-1], ey)
    x, y = v.rods[3].get_data()                        # QUAD4's rod ends on its tip
    ex, ey = v.cam.xy(plate(90.0))
    assert np.isclose(x[-1], ex) and np.isclose(y[-1], ey)


def test_released_carrier_loses_its_rod(run_dir):
    S = tv.load(run_dir)
    assert S['released'] == {1: 5.0}
    v = tv.Video(S, 0.0, 10.0, np.array([4.9, 5.1]), 1.0, 'test', tv.VIEW)
    v.update(0)
    assert len(v.rods[1].get_data()[0]) == 2
    v.update(1)
    assert len(v.rods[1].get_data()[0]) == 0
    assert len(v.rods[0].get_data()[0]) == 2


def test_window_from_events(run_dir):
    ev = tv.load(run_dir)['events']
    assert tv.when('CONTACT-5', ev, 0.0, 10.0) == pytest.approx(-1.0)
    assert tv.when('CONTACT#2+1', ev, 0.0, 10.0) == pytest.approx(7.0)
    assert tv.when('MUX:ours+0.5', ev, 0.0, 10.0) == pytest.approx(7.5)
    assert tv.when('3.5', ev, 0.0, 10.0) == 3.5
    assert tv.when('end', ev, 0.0, 10.0) == 10.0
    for bad in ('NOPE', 'CONTACT#3'):
        with pytest.raises(SystemExit):
            tv.when(bad, ev, 0.0, 10.0)


def test_event_labels():
    labels = [g['label'] for g in tv.event_groups(EVENTS)]
    assert labels == ['contact', 'detach', 'contact', 'attach confirmed, hand-over', 'give-up', 'LAND', 'landed']
    back = [(131.397, 'GIVE_UP', ''), (131.397, 'TAKE_BACK', ''), (131.397, 'MUX:ours', ''),
            (131.570, 'GIVE_UP', '')]
    assert [g['label'] for g in tv.event_groups(back)] == ['give-up, take-back']
    assert [g['label'] for g in tv.event_groups(EVENTS, 5.5, 8.5)] == ['contact', 'attach confirmed, hand-over',
                                                                       'give-up']


def test_frame_times():
    tf = tv.frame_times(10.0, 12.0, 2.0, 30.0)
    assert len(tf) == 31 and tf[-1] == pytest.approx(12.0)
    assert len(tv.frame_times(10.0, 12.0, 1.0, 30.0, decimate=2)) == 31


def test_still_frame_is_full_hd(run_dir, tmp_path):
    out = tmp_path / 'still.mp4'
    assert tv.main([run_dir, '--out', str(out), '--still', '6.5']) == 0
    import matplotlib.pyplot as plt
    img = plt.imread(str(tmp_path / 'still_t0006.50.png'))
    assert img.shape[:2] == (1080, 1920)


@pytest.mark.skipif(not (os.path.exists('/usr/bin/ffmpeg') or shutil.which('ffmpeg')), reason='no ffmpeg')
def test_short_video_encodes(run_dir, tmp_path):
    out = tmp_path / 'clip.mp4'
    assert tv.main([run_dir, '--t0', 'CONTACT', '--t1', 'CONTACT+0.5', '--fps', '10', '--out', str(out)]) == 0
    assert out.stat().st_size > 0
    if shutil.which('ffprobe'):
        info = subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                                        'stream=codec_name,width,height,nb_frames', '-of', 'csv=p=0',
                                        str(out)]).decode().strip()
        assert info == 'h264,1920,1080,6'
