"""
Video recording of a headless Gazebo run (opt-in: `record:` in a config, or run_experiment.py --record).

The run itself renders nothing. Gazebo logs its state (--record-path, --record-period) and the
world it flies is a copy of the config's world with static camera models added and the Sensors
system taken out, so the cameras exist in the log but no frame is drawn while the controllers fly.
tools/render_video.py replays the log afterwards in a headless server that does load the Sensors
system, and pipes each camera's frames (stamped in sim time) into ffmpeg: 30 frames per sim second.
The camera models have to be in the log from the first step: an entity spawned into a playback
takes an id the log may reuse later.
"""
import math
import os
import re

TOPIC_PREFIX = '/mdc_rec/'

# Rig-twin M1 scene: ring at the origin (1.0 m hover), carriers at 270/30/150 deg, QUAD4 on plate 3
# (+y, 0.225 m), Tejen's ball at (1.0, 1.6) and his transit up to ~3.1 m. wide: all of it, floor start
# included; ring: the ring, the carriers, the drop into the net and the re-attach; plate9: the attach (named
# in the 8-14 Oct clockwise numbering; the key stays for the recorded videos).
DEFAULT_CAMERAS = {
    'wide': {'pos': [5.0, -3.1, 3.4], 'look_at': [0.1, 0.35, 1.6], 'hfov_deg': 58.0,
             'size': [1920, 1080]},
    'ring': {'pos': [2.15, -2.1, 3.1], 'look_at': [0.0, 0.05, 1.7], 'hfov_deg': 57.0,
             'size': [1920, 1080]},
    'plate9': {'pos': [0.7, 2.1, 1.55], 'look_at': [0.0, 0.2, 1.35], 'hfov_deg': 60.0,
               'size': [1920, 1080]},
}
DEFAULTS = {'period_s': 0.01, 'fps': 30, 'render': True, 'lead_s': 1.0, 'tail_s': 1.0}
CAMERA_KEYS = {'pos', 'look_at', 'hfov_deg', 'size'}


def parse(raw):
    """The `record:` block -> dict, or None when recording is off. `true` takes every default; a
    camera given by name only (`cameras: [wide]`) takes that default camera."""
    if raw is None or raw is False:
        return None
    if raw is True:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError(f'record: expected true or a mapping, got {raw!r}')
    unknown = set(raw) - set(DEFAULTS) - {'cameras'}
    if unknown:
        raise ValueError(f'record: unknown keys {sorted(unknown)}')
    spec = {k: raw.get(k, v) for k, v in DEFAULTS.items()}
    spec['period_s'] = float(spec['period_s'])
    spec['fps'] = int(spec['fps'])
    if not 0.0 < spec['period_s'] <= 1.0 / spec['fps']:
        raise ValueError(f'record: period_s {spec["period_s"]} must be in (0, 1/fps]: a video frame '
                         f'would repeat a logged state')
    cams = raw.get('cameras', DEFAULT_CAMERAS)
    if isinstance(cams, list):
        missing = [c for c in cams if c not in DEFAULT_CAMERAS]
        if missing:
            raise ValueError(f'record: no default camera {missing}; give pos/look_at')
        cams = {c: DEFAULT_CAMERAS[c] for c in cams}
    spec['cameras'] = {}
    for name, cam in (cams or {}).items():
        if not re.fullmatch(r'[A-Za-z0-9_]+', str(name)):
            raise ValueError(f'record: camera name {name!r} must be [A-Za-z0-9_]+')
        cam = dict(DEFAULT_CAMERAS.get(name, {}), **(cam or {}))
        bad = set(cam) - CAMERA_KEYS
        if bad or not {'pos', 'look_at'} <= set(cam):
            raise ValueError(f'record: camera {name}: needs pos and look_at, knows {sorted(CAMERA_KEYS)}')
        cam.setdefault('hfov_deg', 60.0)
        cam.setdefault('size', [1920, 1080])
        w, h = (int(v) for v in cam['size'])
        if w % 2 or h % 2:
            raise ValueError(f'record: camera {name}: size {w}x{h} must be even (yuv420p)')
        if math.dist(cam['pos'], cam['look_at']) < 1e-3:
            raise ValueError(f'record: camera {name}: pos and look_at coincide')
        spec['cameras'][name] = {'pos': [float(v) for v in cam['pos']],
                                 'look_at': [float(v) for v in cam['look_at']],
                                 'hfov_deg': float(cam['hfov_deg']), 'size': [w, h]}
    if not spec['cameras']:
        raise ValueError('record: no cameras')
    return spec


def camera_pose(pos, look_at):
    """SDF pose (x y z roll pitch yaw) of a camera at pos looking at look_at, no roll."""
    dx, dy, dz = (b - a for a, b in zip(pos, look_at))
    return (*pos, 0.0, -math.atan2(dz, math.hypot(dx, dy)), math.atan2(dy, dx))


def camera_model(name, cam, fps):
    x, y, z, r, p, yaw = camera_pose(cam['pos'], cam['look_at'])
    w, h = cam['size']
    return f'''
    <model name="mdc_rec_{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {r:.4f} {p:.6f} {yaw:.6f}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>1</always_on>
          <update_rate>{fps}</update_rate>
          <visualize>false</visualize>
          <topic>{TOPIC_PREFIX}{name}</topic>
          <camera>
            <horizontal_fov>{math.radians(cam["hfov_deg"]):.6f}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>100</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
'''


SENSORS_PLUGIN = re.compile(
    r'[ \t]*<plugin\b[^>]*name="[^"]*systems::Sensors"[^>]*?(/>|>.*?</plugin>)[ \t]*\n?', re.S)


def recording_world(text, spec):
    """The world text with the camera models added and the Sensors system removed. Every rendering
    sensor in the world would otherwise draw frames in the live server; the source world has none
    (refused below), so without one the system had nothing to do."""
    if re.search(r'<sensor\b[^>]*type="(camera|depth_camera|rgbd_camera|gpu_lidar|gpu_ray|thermal|'
                 r'segmentation|boundingbox_camera|wideanglecamera)"', text):
        raise ValueError('record: the world already has a rendering sensor; the recording copy would '
                         'drop the Sensors system it needs')
    out, n = SENSORS_PLUGIN.subn('', text)
    cams = ''.join(camera_model(k, c, spec['fps']) for k, c in spec['cameras'].items())
    i = out.rfind('</world>')
    if i < 0:
        raise ValueError('record: no </world> in the world file')
    return out[:i] + cams + '  ' + out[i:]


def write_recording_world(src, dst, spec):
    with open(src) as fh:
        text = fh.read()
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, 'w') as fh:
        fh.write(recording_world(text, spec))
    return dst
