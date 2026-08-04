"""Structural tests for the generated RViz config.

Written after a real failure on 2026-08-04: a refactor dropped the top-level
`Window Geometry:` key, so `Height`/`Width`/`Displays`/`ArmPanel` fell inside
`Visualization Manager:` -- which already has a `Displays:` key. YAML keeps the LAST
duplicate, so the entire display list was silently replaced by `{collapsed: false}`
and RViz opened completely empty. Every process started, every log line looked
healthy, and nothing said the config was wrong.

The lesson these tests encode: a config file that is *generated* has to be *parsed*
in a test, not eyeballed.

    python3 -m pytest src/controller_quad_load/test/test_rviz_config.py -v
"""
import pytest
import yaml

from controller_quad_load.rviz_config import build_config, drone_colour, drone_mesh_colour


class _NoDuplicateKeyLoader(yaml.SafeLoader):
    """yaml.safe_load silently accepts duplicate keys and keeps the last -- which is
    exactly how the bug hid. This loader raises instead."""


def _no_duplicates(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise AssertionError(f"duplicate key {key!r} in the generated RViz config")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_NoDuplicateKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicates)


def parse(n=3, **kw):
    return yaml.load(build_config(n, **kw), Loader=_NoDuplicateKeyLoader)


# ── the bug that motivated this file ─────────────────────────────────────────

def test_config_has_no_duplicate_keys():
    """The failure mode: a second `Displays:` key wiping the display list."""
    parse(3)


def test_window_geometry_is_top_level():
    cfg = parse(3)
    assert 'Window Geometry' in cfg, (
        "Window Geometry must sit at column 0; nested, its keys collide with "
        "Visualization Manager's own Displays key and blank the whole scene")
    assert set(cfg['Window Geometry']) >= {'Height', 'Width'}


def test_displays_is_a_populated_list():
    vm = parse(3)['Visualization Manager']
    assert isinstance(vm['Displays'], list)
    assert len(vm['Displays']) > 5


# ── content ──────────────────────────────────────────────────────────────────

def test_every_drone_gets_an_airframe_and_a_plan():
    n = 4
    names = [d.get('Name', '') for d in parse(n)['Visualization Manager']['Displays']]
    for i in range(n):
        assert f'Drone {i} airframe' in names
        assert f'Drone {i} MPC plan' in names


def test_display_count_scales_with_fleet_size():
    def count(n):
        return len(parse(n)['Visualization Manager']['Displays'])
    # 4 displays per drone (airframe, plan, actual, reference)
    assert count(4) - count(3) == 4


def test_payload_is_always_shown():
    names = [d.get('Name', '') for d in parse(2)['Visualization Manager']['Displays']]
    assert 'Payload box' in names
    assert 'Payload desired' in names


def test_arm_panel_declares_the_fleet_size():
    panels = parse(3)['Panels']
    arm = next(p for p in panels if p['Class'].endswith('ArmPanel'))
    assert arm['NumDrones'] == 3


@pytest.mark.parametrize('detach,attach', [(False, False), (True, False), (True, True)])
def test_panel_button_flags_are_honoured(detach, attach):
    panels = parse(3, detach=detach, attach=attach)['Panels']
    arm = next(p for p in panels if p['Class'].endswith('ArmPanel'))
    assert arm['ShowDetach'] is detach
    assert arm['ShowAttach'] is attach


def test_sim_and_real_differ_only_in_the_panel_flags():
    """The whole point of the shared module (F8): the hardware view must not drift
    into being the degraded one, as it had."""
    sim = build_config(3, show_actual=False, detach=True, attach=True)
    real = build_config(3, show_actual=False, detach=True, attach=True)
    assert sim == real


def test_colours_are_stable_and_wrap():
    """Drone colours are shared with the analysis plots, so drone 2 is the same
    colour in RViz and in every thesis figure."""
    assert drone_colour(0) != drone_colour(1)
    assert drone_colour(0) == drone_colour(5)          # palette of 5, wraps
    assert drone_mesh_colour(0).count(' ') == 3        # "r g b a"
