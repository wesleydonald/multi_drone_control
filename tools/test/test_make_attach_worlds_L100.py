"""The 1 m attach family (tools/make_attach_worlds_L100.py; every twin attach world since 22 Oct): the files on disk are
current, every world flies 1 m carriers and QUAD4's 1 m arm on the M1 layout, the pickup world and model of 15 Oct are
unchanged, and generating the family leaves make_attach_worlds' own 0.55 m settings as they were.

    python3 -m pytest tools/test/test_make_attach_worlds_L100.py -q
"""
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
ASSETS = REPO / 'simulation_assets'
sys.path.insert(0, str(REPO / 'tools'))

import make_attach_worlds as maw                                  # noqa: E402
import make_attach_worlds_L100 as mal                             # noqa: E402
from check_geometry import world_geometry                         # noqa: E402

WORLDS = [w.replace('.sdf', '_L100.sdf') for w in
          ('three_rigid_ground_rig_attach.sdf', 'three_rigid_ground_rig_attach_welded.sdf',
           'three_rigid_ground_rig_attach_pickup.sdf', 'three_rigid_ground_rig_attach_welded_pickup.sdf',
           'three_rigid_ground_rig_attach_held_pickup.sdf', 'three_rigid_ground_rig_attach_push.sdf',
           'three_rigid_ground_rig_attach_pickup_push.sdf', 'three_rigid_ground_rig_attach_welded_pickup_push.sdf')]


def test_the_1m_family_on_disk_is_current():
    r = subprocess.run([sys.executable, str(REPO / 'tools' / 'make_attach_worlds_L100.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_every_1m_world_flies_1m_rods_and_quad4s_1m_arm_on_the_m1_layout():
    for w in WORLDS:
        g = world_geometry(ASSETS / w)
        assert (g['cable_len'], g['attach_arm_len'], g['attach_arm_pivot_z']) == (1.0, 1.0, -0.04), w
        assert g['attach_azimuths'] == [30.0, 150.0, 270.0] and g['num_drones'] == 3, w


def test_a_generation_leaves_the_0p55m_settings_as_they_were(tmp_path):
    before = (maw.CABLE_LEN, list(maw.RIG_ARGS), maw.BALL_XY, dict(maw.MODELS), dict(maw.PICKUP_MODELS))
    written = mal.generate(str(tmp_path))
    assert (maw.CABLE_LEN, list(maw.RIG_ARGS), maw.BALL_XY, dict(maw.MODELS), dict(maw.PICKUP_MODELS)) == before
    assert sorted(w for w in written if not w.startswith('models')) == sorted(WORLDS)
    assert 'models/rig_pickup_ball.sdf' not in written                     # shared with the 0.55 m family
