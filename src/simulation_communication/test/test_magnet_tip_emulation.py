"""The rig's magnet marker in the twin (card 2026-10-21_tip_detect): the emulator reports the payload end of each
carrier's rod, from the rod model's pose (link frame at the rod's middle, z along it, the magnet at -L/2), and the
chain's worlds publish those poses."""
import os
import re
import subprocess
import sys

import numpy as np
import pytest

from simulation_communication.payload_mocap_emulator import PayloadMocapEmulator as E

ASSETS = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'simulation_assets')


def test_the_rod_end_is_half_its_length_down_its_axis():
    assert np.allclose(E.rod_end((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), 0.275), (1.0, 2.0, 2.725))
    s = np.sin(np.pi / 4)
    assert np.allclose(E.rod_end((0.0, 0.0, 1.0), (s, 0.0, 0.0, s), 0.5), (0.0, 0.5, 1.0))   # 90 deg about x


@pytest.mark.skipif(not os.path.isdir(ASSETS), reason='simulation_assets not beside the package')
def test_the_generator_publishes_each_detachable_rod_only_when_asked(tmp_path):
    def world(*extra):
        out = tmp_path / f'w{len(extra)}.sdf'
        subprocess.run([sys.executable, 'generate_rigid_world.py', '--n', '2', '--cable-len', '0.55', '--detachable',
                        *extra, '--out', str(out)], check=True, cwd=ASSETS, stdout=subprocess.DEVNULL)
        return out.read_text()
    plain, pub = world(), world('--tether-pose-hz', '60')
    tethers = re.findall(r'<model name="tether_\d">.*?</model>', pub, re.S)
    assert len(tethers) == 2 and all('PosePublisher' in t and '<update_frequency>60</update_frequency>' in t
                                     for t in tethers)
    assert not any('PosePublisher' in t for t in re.findall(r'<model name="tether_\d">.*?</model>', plain, re.S))


def test_the_ring_tilt_fault_turns_about_the_rings_y_and_the_drones_about_x():
    """MOCAP_TILT -1 (card 2026-10-21_tip_detect rev 3): the ring's mis-solve turns about its y, the T's 3-9 axis, so
    plates 0 and 6 (on x) move; a drone's tilt fault stays about its x."""
    from tf_transformations import quaternion_matrix
    R = quaternion_matrix(E._axis_q((0.0, 12.0, 0.0), 1.0, 1))[:3, :3]
    assert np.allclose(R @ [0.0, 1.0, 0.0], [0.0, 1.0, 0.0])
    assert np.degrees(np.arccos((R @ [1.0, 0.0, 0.0])[0])) == pytest.approx(12.0)
    half = E._axis_q((0.0, 12.0, 0.4), 0.2, 1)                       # half way up a 0.4 s ramp
    assert np.degrees(2 * np.arccos(half[3])) == pytest.approx(6.0)
    Rx = quaternion_matrix(E._axis_q((0.0, 12.0, 0.0), 1.0, 0))[:3, :3]
    assert np.allclose(Rx @ [1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
