"""fleet_viz: the swung magnet in RViz (the tip sphere coloured by the coil and the weld, the tether from QUAD4's
body, the puck), the plate labels clockwise, and /fleet/mocap_summary's per-body age, rate, gap and speed."""
import json
import math

import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from interfaces.msg import MotionCaptureState
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from simulation_communication.fleet_viz import (MARKER_LIFETIME_S, PICKUP_SIZE, TIP_ATTACHED,
                                                TIP_DIAMETER, TIP_OFF, TIP_ON, BodyTrack, FleetViz, StreamStats,
                                                coil_word, mocap_summary, swung_magnet_markers, tilt_deg)


def _pose(x, y, z, q=(1.0, 0.0, 0.0, 0.0)):
    w, qx, qy, qz = (float(v) for v in q)
    return Pose(position=Point(x=float(x), y=float(y), z=float(z)), orientation=Quaternion(w=w, x=qx, y=qy, z=qz))


def _xyz(p):
    return p.x, p.y, p.z


def _rgb(m):
    return m.color.r, m.color.g, m.color.b


ROLL_90 = (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)     # 90 deg about x


def test_tip_tether_and_pickup_plate_with_their_namespaces_types_sizes_and_lifetime():
    tip, body, puck = _pose(0.1, 0.2, 1.0), _pose(0.1, 0.2, 1.6), _pose(1.0, 1.6, 0.0125, ROLL_90)
    ms = swung_magnet_markers(Time(sec=5), tip, body, puck)
    assert [(m.ns, m.type) for m in ms] == [('magnet_tip', Marker.SPHERE), ('tether', Marker.LINE_STRIP),
                                            ('puck', Marker.CUBE)]
    for m in ms:
        assert m.header.frame_id == 'map' and m.action == Marker.ADD and m.header.stamp.sec == 5
        assert m.lifetime.sec + 1e-9 * m.lifetime.nanosec == pytest.approx(MARKER_LIFETIME_S)
    sphere, line, plate = ms
    assert _xyz(sphere.pose.position) == pytest.approx((0.1, 0.2, 1.0))
    assert (sphere.scale.x, sphere.scale.y, sphere.scale.z) == pytest.approx((TIP_DIAMETER,) * 3)
    assert len(line.points) == 2 and 0.0 < line.scale.x < 0.01
    assert _xyz(line.points[0]) == pytest.approx((0.1, 0.2, 1.57))           # 3 cm below the body centre
    assert _xyz(line.points[1]) == pytest.approx((0.1, 0.2, 1.0))
    assert _xyz(plate.pose.position) == pytest.approx((1.0, 1.6, 0.0125))
    assert (plate.scale.x, plate.scale.y, plate.scale.z) == pytest.approx((0.100, 0.100, 0.025)) == PICKUP_SIZE
    o = plate.pose.orientation
    assert (o.w, o.x, o.y, o.z) == pytest.approx(ROLL_90)                   # oriented by its pose


def test_the_tether_leaves_the_body_through_its_attitude():
    _, line = swung_magnet_markers(Time(), _pose(0.0, 0.0, 1.0), _pose(0.5, 0.0, 1.5, ROLL_90))
    assert _xyz(line.points[0]) == pytest.approx((0.5, 0.03, 1.5))           # body z rolled onto world -y: anchor +y


def test_tip_colour_grey_off_orange_on_green_attached():
    def colour(**kw):
        return _rgb(swung_magnet_markers(Time(), _pose(0, 0, 1), **kw)[0])
    assert colour() == pytest.approx(TIP_OFF)
    assert colour(coil_on=True) == pytest.approx(TIP_ON)
    assert colour(attached=True) == pytest.approx(TIP_ATTACHED)
    assert colour(coil_on=True, attached=True) == pytest.approx(TIP_ATTACHED)


def test_a_missing_stream_draws_nothing_for_it():
    tip, body, puck = _pose(0, 0, 1), _pose(0, 0, 1.6), _pose(1, 1, 0)
    assert swung_magnet_markers(Time()) == []
    assert [m.ns for m in swung_magnet_markers(Time(), puck=puck, body=body)] == ['puck']   # no tip, no tether
    assert [m.ns for m in swung_magnet_markers(Time(), tip=tip)] == ['magnet_tip']
    zero_q = _pose(1, 1, 0, (0.0, 0.0, 0.0, 0.0))                              # an unset quaternion
    o = swung_magnet_markers(Time(), puck=zero_q)[0].pose.orientation
    assert (o.w, o.x, o.y, o.z) == (1.0, 0.0, 0.0, 0.0)


def test_coil_words():
    assert coil_word('ON') is True and coil_word(' on ') is True and coil_word('1') is True
    assert coil_word('OFF') is False and coil_word('disable') is False
    assert coil_word('NONE') is None and coil_word('') is None


def test_tilt_from_the_quaternion():
    assert tilt_deg(Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)) == pytest.approx(0.0)
    assert tilt_deg(Quaternion(w=math.cos(math.radians(15)), x=math.sin(math.radians(15)), y=0.0,
                               z=0.0)) == pytest.approx(30.0)
    yaw_only = Quaternion(w=math.cos(1.0), x=0.0, y=0.0, z=math.sin(1.0))
    assert tilt_deg(yaw_only) == pytest.approx(0.0, abs=1e-6)
    assert tilt_deg(Quaternion(w=0.0, x=0.0, y=0.0, z=0.0)) is None
    assert tilt_deg(Quaternion(w=float('nan'), x=0.0, y=0.0, z=0.0)) is None


def _feed(stats, t0, t1, dt=0.01, skip=()):
    k = 0
    while t0 + k * dt <= t1 + 1e-9:
        t = round(t0 + k * dt, 6)
        if not any(a < t < b for a, b in skip):
            stats.add(t)
        k += 1


def test_rate_gap_and_age_over_their_windows():
    s = StreamStats()
    assert s.summary(0.0) == (None, 0.0, None)
    _feed(s, 0.0, 3.5, skip=[(1.0, 1.25)])                   # 100 Hz with a 0.25 s hole ending at 1.25
    age, rate, gap = s.summary(3.5)
    assert age == pytest.approx(0.0) and rate == pytest.approx(100.0) and gap == pytest.approx(0.25)
    age, rate, gap = s.summary(4.0)                          # the stream stopped 0.5 s ago: the open gap counts
    assert age == pytest.approx(0.5) and rate == pytest.approx(75.0) and gap == pytest.approx(0.5)
    _feed(s, 4.0, 6.3)                                       # back; the 0.25 s hole ended more than 5 s ago
    age, rate, gap = s.summary(6.3)
    assert rate == pytest.approx(100.0) and gap == pytest.approx(0.5)      # the 3.5 -> 4.0 gap is still in
    _feed(s, 6.31, 9.1)
    assert s.summary(9.1)[2] == pytest.approx(0.01)


def test_the_clock_going_back_starts_the_stream_again():
    s = StreamStats()
    _feed(s, 100.0, 101.0)
    s.add(2.0)
    assert s.summary(2.0) == (pytest.approx(0.0), pytest.approx(0.5), pytest.approx(0.0))


def test_speed_from_the_twist_or_from_two_summaries():
    tr = BodyTrack()
    tr.update(1.0, _pose(0, 0, 1), Twist(linear=Vector3(x=0.3, y=0.4, z=0.0)))
    assert tr.entry(1.0)['speed_mps'] == pytest.approx(0.5)
    tip = BodyTrack()                                        # PoseStamped: no twist
    tip.update(1.0, _pose(0.0, 0, 1))
    assert tip.entry(1.0)['speed_mps'] is None
    tip.update(1.2, _pose(0.1, 0, 1))
    assert tip.entry(1.2)['speed_mps'] == pytest.approx(0.5)
    assert tip.entry(1.4)['speed_mps'] == pytest.approx(0.5)            # nothing new: the last value stays


def test_summary_document_lists_every_body_and_is_strict_json():
    seen, never = BodyTrack(), BodyTrack()
    seen.update(10.0, _pose(0.2, 0.0, 1.234, (math.cos(0.1), math.sin(0.1), 0.0, 0.0)), Twist())
    lost = BodyTrack()
    lost.update(10.0, _pose(float('nan'), 0.0, float('nan')))
    doc = mocap_summary(10.1, {'drone_0': seen, 'tip': never, 'puck': lost})
    assert doc['t'] == pytest.approx(10.1) and set(doc['bodies']) == {'drone_0', 'tip', 'puck'}
    d0 = doc['bodies']['drone_0']
    assert set(d0) == {'z', 'tilt_deg', 'speed_mps', 'age_s', 'rate_hz', 'max_gap_s'}
    assert d0['z'] == pytest.approx(1.234) and d0['tilt_deg'] == pytest.approx(11.5, abs=0.1)
    assert d0['age_s'] == pytest.approx(0.1) and d0['rate_hz'] == pytest.approx(0.5) and d0['speed_mps'] == 0.0
    assert doc['bodies']['tip'] == {'z': None, 'tilt_deg': None, 'speed_mps': None, 'age_s': None,
                                    'rate_hz': 0.0, 'max_gap_s': None}
    assert doc['bodies']['puck']['z'] is None
    json.loads(json.dumps(doc), parse_constant=lambda c: pytest.fail(f'{c} in the summary'))


class _Cap:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _state(x, y, z, q=(1.0, 0.0, 0.0, 0.0)):
    return MotionCaptureState(pose=_pose(x, y, z, q))


def test_node_topics_markers_summary_and_clockwise_plate_labels():
    rclpy.init(args=['--ros-args', '-p', 'num_drones:=4'])
    try:
        node = FleetViz()
        subs = {t for t, _ in node.get_subscriber_names_and_types_by_node('fleet_viz', '/')}
        pubs = {t for t, _ in node.get_publisher_names_and_types_by_node('fleet_viz', '/')}
        assert {'/magnet_tip_pose', '/pickup/motion_capture_state', '/drone_3/magnet_coil',
                '/magnet/object_attached', '/drone_3/motion_capture_state'} <= subs
        assert {'/fleet/mocap_summary', '/fleet/id_markers'} <= pubs
        node.id_pub, node.summary_pub = _Cap(), _Cap()
        node._drone_cb(_state(0.0, 0.8, 1.6), 3)
        node._track('tip', PoseStamped(pose=_pose(0.0, 0.8, 1.0)).pose)
        node._coil_cb(String(data='ON'))
        node._marker_tick()
        arr = node.id_pub.msgs[-1]
        assert [(m.ns, m.type) for m in arr.markers] == [('magnet_tip', Marker.SPHERE),
                                                         ('tether', Marker.LINE_STRIP)]
        assert _rgb(arr.markers[0]) == pytest.approx(TIP_ON)
        node._attached_cb(Bool(data=True))
        node._track('puck', _pose(1.0, 1.6, 0.011), Twist())
        node._marker_tick()
        arr = node.id_pub.msgs[-1]
        assert [m.ns for m in arr.markers] == ['magnet_tip', 'tether', 'puck']
        assert _rgb(arr.markers[0]) == pytest.approx(TIP_ATTACHED)
        node._summary_tick()
        doc = json.loads(node.summary_pub.msgs[-1].data)
        assert set(doc['bodies']) == {'drone_0', 'drone_1', 'drone_2', 'drone_3', 'payload', 'tip', 'puck'}
        assert doc['bodies']['drone_3']['z'] == pytest.approx(1.6) and doc['bodies']['drone_0']['age_s'] is None
        node._payload_cb(_state(0.0, 0.0, 1.0))
        labels = {m.id: m for a in node.id_pub.msgs for m in a.markers if m.ns == 'plate_number'}
        assert sorted(labels) == list(range(12))
        for k, at in ((1, 330.0), (3, 270.0), (9, 90.0)):                  # plate k at 30k deg clockwise
            p = labels[k].pose.position
            assert labels[k].text == str(k)
            assert math.degrees(math.atan2(p.y, p.x)) % 360.0 == pytest.approx(at)
        node.destroy_node()
    finally:
        rclpy.shutdown()
