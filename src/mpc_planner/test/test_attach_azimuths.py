"""Rim attachments placed by hand (Wesley's rig: 3/12/9 o'clock tethers, 6 o'clock
newcomer) instead of an even ring, and the clockwise angles people type and read (15 Oct).
Each test names the property it protects."""
import numpy as np
import pytest

from mpc_planner.geometry import (attach_points, azimuth_slot_assignment, azimuths_param_from_typed,
                                  ccw_to_cw_deg, cw_text, cw_to_ccw_deg, parse_azimuths_deg, plate_cw_deg,
                                  plate_of_azimuth, plate_of_cw_deg, slot_azimuth_errors, slot_error_text,
                                  slot_offset_report, slot_offset_warnings, tejen_plate)


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


# ── typed angles clockwise, the maths counter-clockwise (Wesley 14 Oct) ─────────────────────────────────

def test_clockwise_and_counter_clockwise_are_one_map_both_ways():
    """a = (-c) mod 360: the M1 carriers on plates 3/11/7 are typed 90/330/210 and sit at 270/30/150 inside; QUAD4's
    plate 9 is typed 270 and sits at 90 (+y)."""
    for cw, ccw in ((90, 270), (330, 30), (210, 150), (270, 90), (0, 0), (180, 180), (360, 0), (-0.0, 0.0),
                    (-90, 90), (450, 270)):
        assert cw_to_ccw_deg(cw) == pytest.approx(ccw) and ccw_to_cw_deg(ccw) == pytest.approx(cw % 360)
    assert cw_to_ccw_deg([90, 330, 210]) == [270.0, 30.0, 150.0]
    assert ccw_to_cw_deg((30.0, 90.0, 150.0, 270.0)) == [330.0, 270.0, 210.0, 90.0]
    assert ccw_to_cw_deg(np.array([0.0, 90.0])) == [0.0, 270.0]
    assert cw_to_ccw_deg(0) == 0.0 and str(cw_to_ccw_deg(-0.0)) == '0.0'       # never -0.0
    assert cw_to_ccw_deg(1e-17) == 0.0                                          # never 360.0
    assert ccw_to_cw_deg is cw_to_ccw_deg


def test_plates_and_their_clockwise_angles():
    """angle = 30 x plate, clockwise from +x; Tejen counts counter-clockwise (ours 9 = his 3, both ways)."""
    assert [plate_cw_deg(p) for p in (3, 11, 7, 9, 0, 12)] == [90.0, 330.0, 210.0, 270.0, 0.0, 0.0]
    assert [plate_of_cw_deg(a) for a in (90, 330, 210, 270, 0, 359, 14.9, 15.1)] == [3, 11, 7, 9, 0, 0, 0, 1]
    assert [plate_of_azimuth(a) for a in (270, 30, 150, 90)] == [3, 11, 7, 9]
    assert [tejen_plate(p) for p in (9, 3, 0, 6, 11)] == [3, 9, 0, 6, 1]
    assert all(tejen_plate(tejen_plate(p)) == p for p in range(12))


def test_the_typed_list_becomes_the_node_parameter_in_its_own_format():
    """The planner's attach_azimuths_deg stays counter-clockwise, in the typed number format, so a parameter dump
    reads as before 15 Oct; the even ring passes unchanged."""
    assert azimuths_param_from_typed('90,330,210') == '270,30,150'
    assert azimuths_param_from_typed('330,270,210,90') == '30,90,150,270'
    assert azimuths_param_from_typed('270.0,0.0,180.0,90.0') == '90.0,0.0,180.0,270.0'
    assert azimuths_param_from_typed(' 0, 240 120 ') == '0,120,240'
    assert azimuths_param_from_typed('360,-90') == '0,90'
    assert azimuths_param_from_typed([90.0, 330.0]) == '270.0,30.0'
    for even in ('', 'even', 'none', 'auto', 'EVEN'):
        assert azimuths_param_from_typed(even) == even
    assert azimuths_param_from_typed(None) == ''
    assert parse_azimuths_deg(azimuths_param_from_typed('90,330,210')) == [270.0, 30.0, 150.0]


def test_shown_angles_are_clockwise():
    assert cw_text([270.0, 30.0, 150.0]) == '[90, 330, 210]'
    assert cw_text([270.0, 30.0, 150.0, 82.3], 1) == '[90.0, 330.0, 210.0, 277.7]'


def _drones_at(az_deg, r=0.6):
    return [np.array([r * np.cos(np.radians(a)), r * np.sin(np.radians(a)), 0.1]) for a in az_deg]


def test_a_mirrored_typed_list_is_an_error_that_says_so():
    """The M1 carriers (270/30/150 inside) with the list typed the old way (270,30,150 read as clockwise = 90/330/210
    inside): every drone 60 deg off its slot; the line is severe and names the mirrored list."""
    slot_az = np.radians(parse_azimuths_deg(azimuths_param_from_typed('270,30,150')))
    drones = _drones_at([270.0, 30.0, 150.0])
    slot2drone = azimuth_slot_assignment(drones, np.zeros(2), 3, slot_az=slot_az)
    errs = slot_azimuth_errors(drones, (0.0, 0.0), 0.0, slot2drone, slot_az)
    assert sorted(round(abs(e)) for _, e, _ in errs) == [60, 60, 60]
    report = slot_offset_report(errs)
    assert len(report) == 3 and all(severe for severe, _ in report)
    assert all('MIRRORED' in text and 'clockwise since 15 Oct' in text and 'ring rigid body' in text
               for _, text in report)
    # typed the new way: on the slots, nothing to say
    slot_az = np.radians(parse_azimuths_deg(azimuths_param_from_typed('90,330,210')))
    errs = slot_azimuth_errors(drones, (0.0, 0.0), 0.0, azimuth_slot_assignment(drones, np.zeros(2), 3,
                                                                                  slot_az=slot_az), slot_az)
    assert max(abs(e) for _, e, _ in errs) < 1e-9 and slot_offset_report(errs) == []


def test_one_plate_off_is_a_warning_and_the_mirror_hint_starts_past_30_deg():
    """10-30 deg (one plate, model-f1) warns about the ring body; past 30 in the whole degrees the line shows it is
    severe and names a mirrored list as well."""
    report = slot_offset_report([(0, -30.000000000000057, 3), (1, 20.0, 11), (2, 31.0, 7), (3, 5.0, 9)])
    assert [severe for severe, _ in report] == [False, False, True]
    assert report[0][1] == ('drone 1 sits 30 deg clockwise of plate 3: check the ring rigid body (+x toward plate 0) '
                            'or the magnet plates')
    assert report[2][1].startswith('drone 3 sits 31 deg anticlockwise of plate 7: the typed angles look MIRRORED')
    assert slot_offset_warnings([(0, -30.0, 3), (2, 31.0, 7)]) == [t for _, t in
                                                                   slot_offset_report([(0, -30.0, 3), (2, 31.0, 7)])]


def test_the_slot_error_line_is_clockwise():
    """+ = clockwise of the plate: a drone 2 deg anticlockwise (azimuth error +2) shows -2.0; plates kept."""
    assert slot_error_text([(0, 2.0, 3), (1, -1.25, 11), (2, 0.0, 7)]) == \
        'slot error (deg, + = clockwise): d1 -2.0 (plate 3), d2 +1.2 (plate 11), d3 +0.0 (plate 7)'
