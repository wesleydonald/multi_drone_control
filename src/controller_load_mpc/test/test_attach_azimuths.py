"""Rim attachments placed by hand (Wesley's rig: 3/12/9 o'clock tethers, 6 o'clock
newcomer) instead of an even ring. Each test names the property it protects."""
import numpy as np
import pytest

from controller_load_mpc.geometry import (attach_points, azimuth_slot_assignment,
                                          parse_azimuths_deg)


def test_parse_accepts_string_list_and_empty():
    assert parse_azimuths_deg('0,90,180') == [0.0, 90.0, 180.0]
    assert parse_azimuths_deg('0, 90 ,180') == [0.0, 90.0, 180.0]
    assert parse_azimuths_deg([0, 90, 180]) == [0.0, 90.0, 180.0]
    assert parse_azimuths_deg('') is None
    assert parse_azimuths_deg(None) is None


def test_attach_points_default_is_the_even_ring():
    """'' keeps every existing world and launch byte-identical."""
    even = attach_points(3, 0.25, 0.025)
    explicit = attach_points(3, 0.25, 0.025, '0,120,240')
    for a, b in zip(even, explicit):
        assert np.allclose(a, b)


def test_attach_points_clock_face():
    """3/12/9 o'clock = azimuth 0/90/180 in the load frame."""
    pts = attach_points(3, 0.25, 0.025, '0,90,180')
    assert np.allclose(pts[0][:2], [0.25, 0.0])
    assert np.allclose(pts[1][:2], [0.0, 0.25])
    assert np.allclose(pts[2][:2], [-0.25, 0.0])


def test_attach_points_rejects_wrong_count():
    with pytest.raises(ValueError):
        attach_points(3, 0.25, 0.025, '0,90')


def test_slot_assignment_handles_uneven_slots():
    """Drones parked in any order around 3/12/9 o'clock must each be matched to their
    own rim point -- the cyclic-rotation shortcut only worked for an even ring."""
    slot_az = [0.0, np.pi / 2, np.pi]
    load = np.array([0.0, 0.0])
    # drone 0 at 9 o'clock, drone 1 at 3 o'clock, drone 2 at 12 o'clock
    drones = [np.array([-0.6, 0.02]), np.array([0.6, -0.01]), np.array([0.01, 0.6])]
    perm = azimuth_slot_assignment(drones, load, 3, slot_az=slot_az)
    assert perm == [1, 2, 0]           # slot 0 (3 o'clock) <- drone 1, slot 1 <- drone 2, slot 2 <- drone 0


def test_slot_assignment_respects_load_yaw_with_uneven_slots():
    """The slots rotate with the payload (the 2026-08-03 real-world bug), also when
    they are uneven."""
    slot_az = [0.0, np.pi / 2, np.pi]
    yaw = np.radians(40.0)
    load = np.array([0.3, -0.2])
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    drones = [load + R @ np.array([0.6 * np.cos(a), 0.6 * np.sin(a)]) for a in slot_az]
    shuffled = [drones[2], drones[0], drones[1]]
    perm = azimuth_slot_assignment(shuffled, load, 3, load_yaw=yaw, slot_az=slot_az)
    assert perm == [1, 2, 0]
