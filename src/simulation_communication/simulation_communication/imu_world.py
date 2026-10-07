"""Does a Gazebo world publish the X3 gyros? (sim Betaflight bridges, rate_source imu)

In imu mode a bridge sends no motor command until a gyro sample arrives, so a world whose
Imu system is missing never flies. The system may sit on the world, on an <include>
(Tejen's single-X3 worlds) or inside an included model file (our x3_drone*.sdf); every
flying model (one with a multicopter motor plugin) also needs an imu sensor.
"""
import os
import re
import xml.etree.ElementTree as ET

MOTOR_PLUGIN = 'multicopter-motor-model'


def _parse(path):
    """Root element; gz tolerates '--' inside comments, expat does not, so strip them."""
    with open(path, 'rb') as fh:
        data = fh.read()
    try:
        return ET.fromstring(data)
    except ET.ParseError:
        return ET.fromstring(re.sub(rb'<!--.*?-->', b'', data, flags=re.S))


def _is_imu_system(plugin):
    ident = f"{plugin.get('filename', '')} {plugin.get('name', '')}".lower()
    return 'imu-system' in ident or 'systems::imu' in ident


def _is_motor(plugin):
    return MOTOR_PLUGIN in plugin.get('filename', '').lower()


def _resolve(uri, base_dir):
    """Local file behind an <include><uri>, or None (fuel/remote, or not found)."""
    uri = uri.strip()
    if uri.startswith(('http://', 'https://')):
        return None
    if uri.startswith('file://'):
        uri = uri[len('file://'):]
    if uri.startswith('model://'):
        name = uri[len('model://'):]
        dirs = [base_dir] + [d for d in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if d]
        for d in dirs:
            for cand in (os.path.join(d, name, 'model.sdf'), os.path.join(d, name)):
                if os.path.isfile(cand):
                    return cand
        return None
    path = uri if os.path.isabs(uri) else os.path.join(base_dir, uri)
    if not os.path.exists(path) and not os.path.isabs(uri):
        # a world in old_worlds/ includes models/ of the folder above
        path = os.path.join(os.path.dirname(base_dir), uri)
    if os.path.isdir(path):
        path = os.path.join(path, 'model.sdf')
    return path if os.path.isfile(path) else None


def _flying(model):
    return any(_is_motor(p) for p in model.findall('plugin'))


def _has_imu_sensor(tree):
    return any(s.get('type') == 'imu' for s in tree.iter('sensor'))


def _scan(root, path, found, flying, depth=0):
    """Walk one SDF file and its local includes: the files that load an Imu system into
    `found`, flying models into `flying` as (name, has_imu_sensor)."""
    base_dir = os.path.dirname(path)
    if any(_is_imu_system(p) for p in root.iter('plugin')):
        found.append(path)
    flying += [(m.get('name', '?'), _has_imu_sensor(m))
               for m in root.iter('model') if _flying(m)]
    if depth >= 4:
        return
    for inc in root.iter('include'):
        sub_path = _resolve(inc.findtext('uri') or '', base_dir)
        if sub_path is None:
            continue
        sub_path = os.path.realpath(sub_path)
        sub = _parse(sub_path)
        if _flying(inc):      # motor plugins declared on the include itself
            name = inc.findtext('name') or next(
                (m.get('name') for m in sub.iter('model')), sub_path)
            flying.append((name.strip(), _has_imu_sensor(sub)))
        _scan(sub, sub_path, found, flying, depth + 1)


def imu_report(world_path):
    """(imu_system_sources, flying_models_without_an_imu_sensor) for a world file."""
    world_path = os.path.realpath(world_path)
    found, flying = [], []
    _scan(_parse(world_path), world_path, found, flying)
    return found, sorted({n for n, ok in flying if not ok})


def require_imu_world(world_path, why='rate_source imu'):
    """Raise RuntimeError unless the world loads an Imu system and every flying model
    carries an imu sensor."""
    if not os.path.isfile(world_path):
        raise RuntimeError(f'{why}: world {world_path} not found')
    found, missing = imu_report(world_path)
    if not found:
        raise RuntimeError(
            f'{why} but {world_path} loads no Imu system: its gyros never publish and the '
            f'sim Betaflight bridges send no motor command. Add <plugin filename='
            f'"gz-sim-imu-system" name="gz::sim::systems::Imu"/> to the world, or fly '
            f'rate_source pose')
    if missing:
        raise RuntimeError(f'{why} but these flying models in {world_path} have no imu '
                           f'sensor: {", ".join(missing)}')
    return found


def launch_guard(world_config=None, world_path=None, rate_source_config='rate_source'):
    """OpaqueFunction for a launch that starts gz: refuse a world without the Imu system
    when the launch's rate_source (default imu) is imu. Place it before the gz process."""
    from launch.actions import OpaqueFunction

    def _check(context):
        rate_source = context.launch_configurations.get(rate_source_config, 'imu')
        if rate_source == 'imu':
            path = world_path or context.launch_configurations[world_config]
            require_imu_world(path, why=f'{rate_source_config} imu')
        return []
    return OpaqueFunction(function=_check)
