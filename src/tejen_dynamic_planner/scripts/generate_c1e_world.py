#!/usr/bin/env python3
"""Generate the C.1e Gazebo world from the planner's canonical YAML scene."""

import argparse
from pathlib import Path
import sys

import yaml


PARAMETER_ROOT = 'dynamic_planner_active_commissioning'
SCENE_KEYS = (
    'obstacle_name',
    'obstacle_center_x_m',
    'obstacle_center_y_m',
    'obstacle_center_z_m',
    'obstacle_half_x_m',
    'obstacle_half_y_m',
    'obstacle_half_z_m',
    'obstacle_yaw_rad',
)


def load_scene(config_path):
    with config_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    try:
        parameters = document[PARAMETER_ROOT]['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise ValueError('C.1e config is missing the ROS parameter root') from exc
    missing = [key for key in SCENE_KEYS if key not in parameters]
    if missing:
        raise ValueError('C.1e config is missing: ' + ', '.join(missing))
    if parameters.get('static_scene_enabled') is not True:
        raise ValueError('C.1e static_scene_enabled must be true')
    for key in ('obstacle_half_x_m', 'obstacle_half_y_m', 'obstacle_half_z_m'):
        if float(parameters[key]) <= 0.0:
            raise ValueError(f'{key} must be positive')
    return parameters


def obstacle_model(scene):
    size = ' '.join(str(2.0 * float(scene[key])) for key in (
        'obstacle_half_x_m', 'obstacle_half_y_m', 'obstacle_half_z_m'))
    pose = ' '.join(str(float(scene[key])) for key in (
        'obstacle_center_x_m', 'obstacle_center_y_m', 'obstacle_center_z_m'))
    pose += f" 0 0 {float(scene['obstacle_yaw_rad'])}"
    name = str(scene['obstacle_name'])
    if not name or any(character in name for character in '<>"\''):
        raise ValueError('obstacle_name contains an unsupported XML character')
    return f"""
    <!-- AUTO-GENERATED from tejen_dynamic_planner/config/c1e_static_sim.yaml. -->
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>0.95 0.35 0.05 1</ambient>
            <diffuse>0.95 0.35 0.05 1</diffuse>
            <specular>0.15 0.15 0.15 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--base-world', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()

    scene = load_scene(args.config)
    base = args.base_world.read_text(encoding='utf-8')
    # The legacy world's introductory XML comment contains the command-line
    # token "--msgtype". XML comments may not contain "--", even though Gazebo
    # has historically accepted this file. Sanitise only the generated copy.
    base = base.replace('--msgtype', '[msgtype]')
    marker = '\n  </world>'
    if base.count(marker) != 1:
        raise ValueError('base world must contain exactly one closing world tag')
    if f'<model name="{scene["obstacle_name"]}">' in base:
        raise ValueError('base world already contains the C.1e obstacle')
    generated = base.replace(marker, obstacle_model(scene) + marker)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(generated, encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # command-line validation should fail loudly
        print(f'generate_c1e_world.py: FAIL: {exc}', file=sys.stderr)
        raise SystemExit(1)
