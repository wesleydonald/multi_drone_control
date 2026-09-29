#!/usr/bin/env python3
"""M2 hand-over bench world (G5, GOALS step 3): Tejen's four X3s already welded to the
0.86 kg ring at his M2D success-hold pose, each held in the air by a detachable hanger,
so our takeover (handover -> creep -> lift -> hover -> LAND) runs without his 10-15 min join.

    python3 tools/sim_test/make_m2_bench_world.py                 # T0019 hold (default)
    python3 tools/sim_test/make_m2_bench_world.py --evidence logs/m2_attachment/<run> --t 104.5

Writes TEST COPIES under simulation_assets/tejen/bench_m2/ (his shipped SDFs are only read):
  m2_bench_world.sdf, m2_bench_ring.sdf, m2_bench_x3_{0..3}.sdf, m2_bench_manifest.json,
  and m2_bench_world_imu.sdf (the same world plus the Imu system, for rate_source:=imu).

How it is built:
  * his generator (tejen_mission.m2c_ground_spawn.generate_m2c_assets, imported read-only)
    makes the M2C world, ring and X3 copies in a temp dir; the bench keeps his world
    skeleton (pose publishers, motor plugins, sensor-system removal, collision bitmasks)
    and his four ring->magnet DetachableJoints;
  * the ring gets the runner's handover patch (run_m2d_sequential_attachment.sh: dynamic,
    0.86 kg) and rests on its legs at z 0.100 (the legs reach 0.100 below the origin; his
    runner spawns it at 0.035 and the contact solver lifts it to 0.100 in ~65 s);
  * each magnet centre sits on its plate (sphere on the plate top + --weld-gap-m) and the
    rod points back along the logged direction, which fixes the body pose; the X3 copies
    are re-made with his _write_vehicle_copy for that pose;
  * a DetachableJoint only attaches, it never releases on its own: nothing in the bench
    sends /drone_i/magnet/detach, so the magnets stay welded from the first step (the
    runner's paused startup release is not run);
  * one hanger per X3 (a link welded to the world + DetachableJoint to X3/base_link,
    attached at the first step), released by gz.msgs.Empty on /bench/hanger_i/detach.
    --hanger static (static model) and --hanger kinematic (the m2a_x3_support.sdf pattern)
    are kept for comparison; see hanger_model for why neither is the default;
  * his M2C world strips the Sensors/Imu systems (the X3 keeps an inert 1000 Hz imu_sensor),
    so the gyro publishes only in the _imu copy, which adds one world-level Imu system.
"""
import argparse
import csv
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'src', 'tejen_mission'))
from tejen_mission import m2c_ground_spawn as m2c  # noqa: E402
from tejen_mission.m2b_ground_spawn import _format_pose  # noqa: E402
from tejen_mission.m2b_ground_start import quaternion_xyzw_from_two_vectors  # noqa: E402

ASSETS = os.path.join(REPO, 'simulation_assets')
SOURCE_X3 = os.path.join(ASSETS, 'tejen', 'modelLargeM2BallMagnet.sdf')
SOURCE_RING = os.path.join(ASSETS, 'tejen', 'm2a_ring_fixture.sdf')
WORLD_TEMPLATE = os.path.join(ASSETS, 'tejen', 'world_m2b_single_attachment.sdf')
OUT_DIR = os.path.join(ASSETS, 'tejen', 'bench_m2')
MESH = os.path.join(ASSETS, 'tejen', 'tbs_sourceone_v5.stl')

RING_YAW_DEG = m2c.M2C_RING_YAW_DEG
RING_REST_Z = 0.100
PLATE_RADIUS = 0.25          # m2a_ring_fixture.sdf plate_k at 30k deg, top face at local z 0
PLATE_PITCH_DEG = 30.0
MAGNET_RADIUS = m2c.MAGNET_RADIUS_M
# ball joint (pivot) -> magnet centre, and the pivot below the body origin
ROD_REACH = m2c.TETHER_LENGTH_M - abs(m2c.BASE_JOINT_OFFSET_TETHER_Z_M)
PIVOT_BODY_Z = m2c.TETHER_ORIGIN_INITIAL_BODY_Z_M + m2c.BASE_JOINT_OFFSET_TETHER_Z_M
GAP_LIMIT_M = 0.005

# T0019 (results/2026-09-27/T0019_m2_claim_ki5, evidence m2d_sequential_20260927_052404) at
# t=104.50 s sim, when the four muxes switched to our trackers (ours.log 05:39:33.2): drone ->
# (plate, rod elevation at the pivot deg, rod azimuth minus the plate's radial azimuth deg).
# Rod length 0.450 on all four. Drone 3 welded at ~103 s and is still swinging (72.8-73.5 deg).
HOLD_T0019 = {0: (3, 74.39, -1.06), 1: (0, 74.29, 0.70), 2: (6, 73.88, 0.35), 3: (9, 73.51, 0.40)}
HOLD_T0019_SOURCE = 'T0019 m2d_sequential_20260927_052404 t=104.50'


def ring_patch_dynamic(text):
    """The handover patch from run_m2d_sequential_attachment.sh, verbatim in effect."""
    s, n = re.subn(r'<static>\s*true\s*</static>', '<static>false</static>', text, count=1)
    assert n == 1, 'ring <static> not found'
    inertial = ('<inertial><mass>0.86</mass><inertia><ixx>2.733e-02</ixx><ixy>0</ixy><ixz>0</ixz>'
                '<iyy>2.733e-02</iyy><iyz>0</iyz><izz>5.452e-02</izz></inertia></inertial>')
    s, n = re.subn(r'(<link name="payload_link">)', r'\1' + inertial, s, count=1)
    assert n == 1, 'ring payload_link not found'
    return s


def hold_from_evidence(evidence, t_hold):
    """(plate, elevation, azimuth offset) per drone from his attachment.csv at t_hold."""
    out = {}
    for i in m2c.DRONE_IDS:
        path = os.path.join(evidence, f'drone_{i}', 'attachment', 'attachment.csv')
        with open(path) as fh:
            r = min(csv.DictReader(fh), key=lambda r: abs(float(r['ros_time_s']) - t_hold))
        g = lambda k: float(r[k])
        yaw = 2.0 * math.atan2(g('ring_qz'), g('ring_qw'))
        pivot = np.array([g('drone_x'), g('drone_y'), g('drone_z') + PIVOT_BODY_Z])
        u = pivot - np.array([g('magnet_x'), g('magnet_y'), g('magnet_z')])
        u /= np.linalg.norm(u)
        plate = int(r['assigned_plate_id'])
        radial = yaw + math.radians(PLATE_PITCH_DEG * plate)
        daz = (math.atan2(u[1], u[0]) - radial + math.pi) % (2 * math.pi) - math.pi
        out[i] = (plate, math.degrees(math.asin(u[2])), math.degrees(daz))
    return out


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def hold_geometry(drone_id, plate, elev_deg, daz_deg, body_yaw_deg, weld_gap):
    ring = np.array([0.0, 0.0, RING_REST_Z])
    yaw = math.radians(RING_YAW_DEG)
    a = math.radians(PLATE_PITCH_DEG * plate)
    plate_top = ring + rot_z(yaw) @ np.array([PLATE_RADIUS * math.cos(a), PLATE_RADIUS * math.sin(a), 0.0])
    magnet = plate_top + np.array([0.0, 0.0, MAGNET_RADIUS + weld_gap])
    az = yaw + a + math.radians(daz_deg)
    el = math.radians(elev_deg)
    u = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    anchor = magnet + ROD_REACH * u
    body = anchor - np.array([0.0, 0.0, PIVOT_BODY_Z])
    d = -u                                            # pivot -> magnet
    tether_origin = anchor - abs(m2c.BASE_JOINT_OFFSET_TETHER_Z_M) * d
    q = quaternion_xyzw_from_two_vectors(np.array([0.0, 0.0, -1.0]), d)
    geo = m2c.GroundVehicleGeometry(
        drone_id=drone_id, vehicle_id=f'drone_{drone_id}',
        body_position_world=tuple(float(v) for v in body),
        body_yaw_rad=math.radians(body_yaw_deg),
        joint_anchor_world=tuple(float(v) for v in anchor),
        magnet_center_world=tuple(float(v) for v in magnet),
        tether_origin_world=tuple(float(v) for v in tether_origin),
        tether_quaternion_xyzw=tuple(float(v) for v in q),
        cable_angle_from_horizontal_deg=float(elev_deg))
    return geo, plate_top


def _pose_to_T(text):
    x, y, z, r, p, yw = (float(v) for v in text.split())
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    T = np.eye(4)
    T[:3, :3] = rot_z(yw) @ Ry @ Rx
    T[:3, 3] = (x, y, z)
    return T


def check_written_x3(x3_path, geo, plate_top):
    """Rod tip and pivot as the WRITTEN SDF places them, against the plate and the body."""
    model = ET.parse(x3_path).getroot().find('model')
    tether = model.find("./link[@name='tether_rod']/pose")
    assert tether is not None and tether.attrib.get('relative_to') == 'X3/base_link'
    tip = model.find("./link[@name='magnet_tip_link']/pose")
    assert tip is not None and tip.attrib.get('relative_to') == 'tether_rod'
    joint = model.find("./joint[@name='base_to_tether']/pose")
    T_wb = np.eye(4)
    T_wb[:3, :3] = rot_z(geo.body_yaw_rad)
    T_wb[:3, 3] = geo.body_position_world
    T_wt = T_wb @ _pose_to_T(tether.text)
    magnet = (T_wt @ _pose_to_T(tip.text))[:3, 3]
    pivot = (T_wt @ _pose_to_T(joint.text if joint is not None else '0 0 0 0 0 0'))[:3, 3]
    R_ring = rot_z(math.radians(RING_YAW_DEG))
    rel = R_ring.T @ (magnet - plate_top)
    gap = float(rel[2] - MAGNET_RADIUS)
    xy = float(math.hypot(rel[0], rel[1]))
    pivot_err = float(np.linalg.norm(pivot - (np.array(geo.body_position_world)
                                              + np.array([0.0, 0.0, PIVOT_BODY_Z]))))
    return {'magnet_world': [round(float(v), 6) for v in magnet], 'gap_m': gap,
            'plate_xy_err_m': xy, 'pivot_err_m': pivot_err,
            'rod_len_m': float(np.linalg.norm(magnet - pivot))}


def hanger_model(i, geo, kind):
    b = geo.body_position_world
    m = ET.Element('model', name=f'bench_hanger_{i}')
    if kind == 'static':
        ET.SubElement(m, 'static').text = 'true'
    ET.SubElement(m, 'pose').text = _format_pose([b[0], b[1], b[2], 0.0, 0.0, geo.body_yaw_rad])
    link = ET.SubElement(m, 'link', name='hanger_link')
    if kind in ('world', 'kinematic'):
        if kind == 'kinematic':                   # m2a_x3_support.sdf
            ET.SubElement(link, 'kinematic').text = 'true'
        ET.SubElement(link, 'gravity').text = 'false'
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '1.0'
        inertia = ET.SubElement(inertial, 'inertia')
        for k in ('ixx', 'iyy', 'izz'):
            ET.SubElement(inertia, k).text = '0.01'
    if kind == 'world':
        # dartsim welds a FreeJoint-root child (X3/base_link) into the PARENT's skeleton and
        # on detach leaves it there as a new root: a static (immobile) parent would keep the
        # X3 frozen after release, and dartsim ignores <kinematic>. A mobile model welded to
        # the world holds it and lets it go.
        j = ET.SubElement(m, 'joint', name='hanger_to_world', type='fixed')
        ET.SubElement(j, 'parent').text = 'world'
        ET.SubElement(j, 'child').text = 'hanger_link'
    p = ET.SubElement(m, 'plugin', filename='gz-sim-detachable-joint-system',
                      name='gz::sim::systems::DetachableJoint')
    for tag, text in (('parent_link', 'hanger_link'), ('child_model', f'x3_{i}'),
                      ('child_link', 'X3/base_link'),
                      ('detach_topic', f'/bench/hanger_{i}/detach'),
                      ('attach_topic', f'/bench/hanger_{i}/attach'),
                      ('output_topic', f'/bench/hanger_{i}/state')):
        ET.SubElement(p, tag).text = text
    return m


def add_imu_system(world):
    """World-level Imu system after the last world plugin: every X3's imu_sensor publishes
    on /world/<w>/model/x3_i/link/X3/base_link/sensor/imu_sensor/imu (no <topic> set)."""
    last = max(k for k, el in enumerate(world) if el.tag == 'plugin')
    p = ET.Element('plugin', filename='gz-sim-imu-system', name='gz::sim::systems::Imu')
    p.tail = world[last].tail
    world.insert(last + 1, p)


def sha256(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def build(out_dir, hold, hold_source, hanger='world', weld_gap=0.0, gz_check=True):
    if os.environ.get('M2C_TETHER_ORIGIN_BODY_Z_M'):
        raise SystemExit('M2C_TETHER_ORIGIN_BODY_Z_M is set: the bench is for his shipped X3 (0.01)')
    os.makedirs(out_dir, exist_ok=True)
    body_yaw = m2c.DEFAULT_BODY_YAW_DEG     # his MPC holds the spawn yaw (T0019: 12.06/-17.95/33.00/-41.12)
    tmp = tempfile.mkdtemp(prefix='m2_bench_')
    try:
        m2c.generate_m2c_assets(source_x3=SOURCE_X3, source_ring=SOURCE_RING,
                                world_template=WORLD_TEMPLATE, output_dir=tmp,
                                ring_yaw_deg=RING_YAW_DEG,
                                manifest_path=os.path.join(tmp, 'm2c_manifest.json'))
        ring_path = os.path.join(out_dir, 'm2_bench_ring.sdf')
        with open(os.path.join(tmp, 'm2c_ring_fixture.sdf')) as fh:
            ring_text = ring_patch_dynamic(fh.read())
        with open(ring_path, 'w') as fh:
            fh.write(ring_text)

        geos, checks, x3_paths = [], {}, []
        for i in m2c.DRONE_IDS:
            plate, elev, daz = hold[i]
            geo, plate_top = hold_geometry(i, plate, elev, daz, body_yaw[i], weld_gap)
            path = os.path.join(out_dir, f'm2_bench_x3_{i}.sdf')
            m2c._write_vehicle_copy(source_x3=Path(SOURCE_X3), output_path=Path(path), geometry=geo)
            with open(path) as fh:
                txt = fh.read()
            txt = txt.replace('file://tbs_sourceone_v5.stl', 'file://' + MESH)
            with open(path, 'w') as fh:
                fh.write(txt)
            chk = check_written_x3(path, geo, plate_top)
            chk.update(plate=plate, rod_elev_deg=elev, rod_daz_deg=daz,
                       body=[round(v, 6) for v in geo.body_position_world],
                       body_yaw_deg=body_yaw[i])
            if abs(chk['gap_m']) >= GAP_LIMIT_M or chk['plate_xy_err_m'] >= GAP_LIMIT_M:
                raise SystemExit(f'drone {i}: rod tip {1e3 * chk["gap_m"]:.2f} mm above plate '
                                 f'{plate}, {1e3 * chk["plate_xy_err_m"]:.2f} mm off its centre '
                                 f'(limit {1e3 * GAP_LIMIT_M:.0f} mm)')
            if chk['pivot_err_m'] > 1e-6 or abs(chk['rod_len_m'] - ROD_REACH) > 1e-6:
                raise SystemExit(f'drone {i}: pivot/rod geometry off: {chk}')
            geos.append(geo)
            checks[f'drone_{i}'] = chk
            x3_paths.append(path)

        tree = ET.parse(os.path.join(tmp, 'm2c_four_x3_ground.sdf'))
        world = tree.getroot().find('world')
        seen = 0
        for inc in world.findall('include'):
            uri = inc.find('uri')
            if uri.text == 'm2c_ring_fixture.sdf':
                uri.text = ring_path
                inc.find('pose').text = _format_pose(
                    [0.0, 0.0, RING_REST_Z, 0.0, 0.0, math.radians(RING_YAW_DEG)])
                seen += 1
            m = re.fullmatch(r'm2c_x3_(\d)\.sdf', uri.text or '')
            if m:
                i = int(m.group(1))
                uri.text = x3_paths[i]
                b = geos[i].body_position_world
                inc.find('pose').text = _format_pose([b[0], b[1], b[2], 0.0, 0.0, geos[i].body_yaw_rad])
                seen += 1
        if seen != 5:
            raise SystemExit(f'expected 1 ring + 4 X3 includes in his M2C world, patched {seen}')
        for i, geo in enumerate(geos):
            h = hanger_model(i, geo, hanger)
            ET.indent(h, space='  ', level=1)
            h.tail = '\n  '
            world.append(h)
        world_path = os.path.join(out_dir, 'm2_bench_world.sdf')
        tree.write(world_path, encoding='utf-8', xml_declaration=True)
        add_imu_system(world)
        imu_world_path = os.path.join(out_dir, 'm2_bench_world_imu.sdf')
        tree.write(imu_world_path, encoding='utf-8', xml_declaration=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    gz = None
    if gz_check and shutil.which('gz'):
        for path in (world_path, imu_world_path):
            r = subprocess.run(['gz', 'sdf', '-k', path], capture_output=True, text=True,
                               timeout=120)
            gz = (r.stdout + r.stderr).strip()
            if r.returncode != 0 or 'Valid' not in gz:
                raise SystemExit(f'gz sdf -k failed on {path}:\n{gz}')

    manifest = {
        'bench': 'M2 hand-over (GOALS G5 step 3)',
        'hold_source': hold_source,
        'hold': {f'drone_{i}': list(hold[i]) for i in m2c.DRONE_IDS},
        'ring_pose': [0.0, 0.0, RING_REST_Z, 0.0, 0.0, RING_YAW_DEG],
        'ring_mass_kg': 0.86,
        'weld_gap_m': weld_gap,
        'hanger': hanger,
        'imu_world': os.path.basename(imu_world_path),
        'hanger_topics': {f'drone_{i}': {'detach': f'/bench/hanger_{i}/detach',
                                         'attach': f'/bench/hanger_{i}/attach',
                                         'state': f'/bench/hanger_{i}/state'}
                          for i in m2c.DRONE_IDS},
        'magnet_joint_topics': {c.vehicle_id: {'detach': c.detach_topic, 'attach': c.attach_topic,
                                               'state': c.state_topic}
                                for c in m2c.detachable_joint_channels(m2c.DRONE_IDS)},
        'checks': checks,
        'gap_limit_m': GAP_LIMIT_M,
        'gz_sdf_check': gz,
        'sources': {p: sha256(p) for p in (SOURCE_X3, SOURCE_RING, WORLD_TEMPLATE)},
        'generator': os.path.relpath(os.path.abspath(__file__), REPO),
    }
    with open(os.path.join(out_dir, 'm2_bench_manifest.json'), 'w') as fh:
        json.dump(manifest, fh, indent=2)
    return world_path, manifest


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=OUT_DIR)
    ap.add_argument('--evidence', help="his run dir (logs/m2_attachment/m2d_sequential_*)")
    ap.add_argument('--t', type=float, help='sim time of the hold in that run')
    ap.add_argument('--hanger', choices=('world', 'static', 'kinematic'), default='world',
                    help='world: a link welded to the world (default); static/kinematic: '
                         'see hanger_model, both suspected not to release in dartsim')
    ap.add_argument('--weld-gap-m', type=float, default=0.0,
                    help='magnet sphere above the plate top (T0019 welded at 6-7 mm)')
    ap.add_argument('--no-gz-check', action='store_true')
    a = ap.parse_args(argv)
    if (a.evidence is None) != (a.t is None):
        ap.error('--evidence and --t go together')
    if a.evidence:
        hold, src = hold_from_evidence(a.evidence, a.t), f'{a.evidence} t={a.t}'
    else:
        hold, src = HOLD_T0019, HOLD_T0019_SOURCE
    world, man = build(a.out, hold, src, a.hanger, a.weld_gap_m, not a.no_gz_check)
    for k, c in man['checks'].items():
        print(f'{k}: plate {c["plate"]:2d}  body z {c["body"][2]:.3f}  rod {c["rod_elev_deg"]:.1f} deg  '
              f'tip gap {1e3 * c["gap_m"]:+.2f} mm  xy {1e3 * c["plate_xy_err_m"]:.2f} mm')
    print(f'gz sdf -k: {man["gz_sdf_check"] or "skipped"}')
    print(world)
    return 0


if __name__ == '__main__':
    sys.exit(main())
