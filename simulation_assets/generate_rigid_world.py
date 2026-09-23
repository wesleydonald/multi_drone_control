#!/usr/bin/env python3
"""
generate_rigid_world.py
-----------------------
Generate an N-drone RIGID-cable lift world — the reference paper's model
(Sun et al. 2025): massless rigid links, ball joints at both ends. This is the
paper-faithful counterpart of generate_soft_world.py (which makes soft segmented
cables). Rigid cables also remove the soft-cable position-varying-tension effect
that destabilised the MPC tracker.

Geometry matches three_soft_paper.sdf by default (elevated & TAUT: cable_len=1.0,
~45deg elevation), so the planner's cable_len=1.0 already fits.

  - world `quadcopter` (keeps /world/quadcopter/* topics + mocap source)
  - lift_system model with a world-fixed `anchor` canonical link
  - N drones (models/x3_drone{i}.sdf) at equal angular intervals, ELEVATED
  - rigid-body payload RING (500 mm, 12 magnet plates, rim attachments) with the pose publisher
  - N rigid tethers (one cylinder link each), ball joint to payload::body at the
    bottom and to x3_drone{i}::base_link at the top

Usage:
    python3 generate_rigid_world.py --n 3 --out three_rigid.sdf
"""
import argparse

# The real payload (2026-09-23, Te-Jen's M2A ring fixture, m2a_ring_fixture.sdf): a
# 500 mm ring of 24 box segments (60 mm radial x 30 mm thick) carrying 12 magnet plates
# (30 mm radius) every 30 deg on its top face, plate 0 on +x. The tethers attach on the
# plates, so the attach ring radius = plate radius = 0.25 m and the attach plane is the
# plate top. Official mass 0.86 kg (Wesley, 2026-09-23). The fixture's 8 bench legs are
# NOT part of the flying payload (its mocap origin rests 5 cm off the floor, i.e. on the
# ring, not on 6.5 cm legs); --legs adds them for a bench-fixture picture only.
PAYLOAD_RADIUS = 0.25          # plate ring radius = attach radius
RING_SEGMENTS = 24
RING_SEG_LEN = 0.068067840828  # tangential box length (2*pi*R/24 with overlap)
RING_SEG_RADIAL = 0.060
RING_SEG_THICK = 0.030
RING_OUTER = PAYLOAD_RADIUS + RING_SEG_RADIAL / 2.0
RING_INNER = PAYLOAD_RADIUS - RING_SEG_RADIAL / 2.0
PLATES = 12
PLATE_RADIUS = 0.030
PLATE_THICK = 0.005
LEG_RADIUS = 0.010
LEG_LEN = 0.065
LEG_RING_RADIUS = 0.20
ATTACH_PLANE_Z = 0.025         # plate top in the link frame (= the launches' attach_z)
PAYLOAD_MASS = 0.86            # official rig mass, 2026-09-23 (was 0.6)
# ring inertia about the CoG (annulus RING_OUTER/RING_INNER, RING_SEG_THICK thick; the
# plates are 4 % of the volume and sit on the same radius, so they are folded in):
# Ixx = Iyy = m(3(R^2 + r^2) + h^2)/12, Izz = m(R^2 + r^2)/2.
def _inertia(mass):
    rr = RING_OUTER ** 2 + RING_INNER ** 2
    return (mass * (3 * rr + RING_SEG_THICK ** 2) / 12.0, mass * rr / 2.0)
_IXX, _IZZ = _inertia(PAYLOAD_MASS)
import math


def _fmt(x):
    return f"{x:.6f}"


def _rod_body(idx, radius, mass, L, cx, cy, cz, pitch, yaw):
    """The rigid tether cylinder's <inertial>+<visual> body (used both as a plain
    lift_system link and, when detachable, as the single link of a nested tether
    model)."""
    it = mass * L * L / 12.0
    ia = 0.5 * mass * radius * radius
    return f"""
        <pose>{_fmt(cx)} {_fmt(cy)} {_fmt(cz)} 0 {_fmt(pitch)} {_fmt(yaw)}</pose>
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>{it:.3e}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{it:.3e}</iyy><iyz>0</iyz><izz>{ia:.3e}</izz>
          </inertia>
        </inertial>
        <visual name="tether_{idx}_visual">
          <geometry><cylinder><radius>{radius}</radius><length>{L:.4f}</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>"""


def tether_block(idx, drone, attach, radius=0.004, mass=0.001,
                 damping=0.1, detachable=False):
    """One rigid tether (payload bottom -> drone top). Both ends are ball joints (a
    two-force rigid link: the paper model), so the drone's ATTITUDE is decoupled from
    the rod at the top AND the rod swings freely at the payload -- the MPC tilts the
    drone to vector its thrust while the rod pivots independently at both ends.

    Non-detachable: the rod is a plain lift_system link with a payload-end and a
    drone-end ball joint.

    Detachable (drone flies away WITH its cable, payload left clean): release must
    happen at the PAYLOAD end, and Gazebo's only release mechanism (DetachableJoint)
    (a) makes a FIXED weld and (b) requires the released body to be a *model*. So:
      * the rod becomes its own nested MODEL `tether_{idx}` (so it can be a detach child),
      * a near-massless payload-side STUB link is ball-jointed to the payload (this keeps
        the payload-end PIVOT: welding the rod straight to the payload would lock the rod
        upright and break flight, exactly as a drone-end weld would),
      * the DetachableJoint (emitted at model level) welds that stub to the rod model,
      * the drone-end stays a ball joint straight to the drone.
    During flight this is identical to the plain two-ball rod (both ends pivot). On
    /drone_{idx}/detach the stub<->rod weld releases, so the rod + drone (still joined by
    the drone-end ball) fly off together; only the tiny stub stays on the payload."""
    dx = drone[0] - attach[0]
    dy = drone[1] - attach[1]
    dz = drone[2] - attach[2]
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    ux, uy, uz = dx / L, dy / L, dz / L
    pitch = math.acos(max(-1.0, min(1.0, uz)))   # cylinder local-z -> cable dir
    yaw = math.atan2(uy, ux)
    cx, cy, cz = (attach[0] + drone[0]) / 2.0, (attach[1] + drone[1]) / 2.0, \
                 (attach[2] + drone[2]) / 2.0
    half = L / 2.0
    rod = _rod_body(idx, radius, mass, L, cx, cy, cz, pitch, yaw)

    if detachable:
        # Payload-side stub (stays on the payload after detach), rod as a nested model,
        # and a drone-end ball joint. The stub<->rod weld is the DetachableJoint
        # (detachable_joint_block). Ball-joint poses are omitted so they default to the
        # child link origin: payload_to_stub -> attach point, tether_to_drone -> drone
        # (= rod top) -- avoids scoped relative_to across nested models.
        return f"""
      <!-- ===== TETHER {idx}: payload -> drone{idx} (rigid, len={L:.4f}, detach@payload) ===== -->
      <link name="stub_pay_{idx}">
        <pose>{_fmt(attach[0])} {_fmt(attach[1])} {_fmt(attach[2])} 0 0 0</pose>
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>1e-8</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-8</iyy><iyz>0</iyz><izz>1e-8</izz></inertia>
        </inertial>
      </link>
      <joint name="payload_to_stub_{idx}" type="ball">
        <parent>payload::body</parent>
        <child>stub_pay_{idx}</child>
        <axis><xyz>1 0 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis>
        <axis2><xyz>0 1 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis2>
      </joint>
      <model name="tether_{idx}">
        <pose>0 0 0 0 0 0</pose>
        <link name="rod">{rod}
        </link>
      </model>
      <joint name="tether_{idx}_to_drone{idx}" type="ball">
        <parent>tether_{idx}::rod</parent>
        <child>x3_drone{idx}::base_link</child>
        <axis><xyz>1 0 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis>
        <axis2><xyz>0 1 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis2>
      </joint>"""

    return f"""
      <!-- ===== TETHER {idx}: payload -> drone{idx} (rigid, len={L:.4f}) ===== -->
      <link name="tether_{idx}">{rod}
      </link>
      <joint name="payload_to_tether_{idx}" type="ball">
        <parent>payload::body</parent>
        <child>tether_{idx}</child>
        <pose relative_to="tether_{idx}">0 0 {-half:.4f} 0 0 0</pose>
        <axis><xyz>1 0 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis>
        <axis2><xyz>0 1 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis2>
      </joint>
      <joint name="tether_{idx}_to_drone{idx}" type="ball">
        <parent>tether_{idx}</parent>
        <child>x3_drone{idx}::base_link</child>
        <pose relative_to="tether_{idx}">0 0 {half:.4f} 0 0 0</pose>
        <axis><xyz>1 0 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis>
        <axis2><xyz>0 1 0</xyz><dynamics><damping>{damping}</damping></dynamics></axis2>
      </joint>"""


def detachable_joint_block(idx):
    """A DetachableJoint plugin welding the payload-side stub to the tether ROD model
    (a nested model -- see tether_block), released on /drone_{idx}/detach. Because the
    stub is the parent and the rod model is the child, releasing it drops the whole rod
    (and, via the retained drone-end ball joint, the drone) away from the payload, so the
    drone flies off carrying its cable and the payload is left clean. Emitted at the
    lift_system model level (which owns both stub_pay_{idx} and the tether_{idx} model).
    Re-attach on /drone_{idx}/attach is available too, for the future reattach step."""
    return f"""
      <!-- ===== DETACH {idx}: release cable+drone{idx} from the payload (payload left clean) ===== -->
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>stub_pay_{idx}</parent_link>
        <child_model>tether_{idx}</child_model>
        <child_link>rod</child_link>
        <detach_topic>/drone_{idx}/detach</detach_topic>
        <attach_topic>/drone_{idx}/attach</attach_topic>
        <output_topic>/drone_{idx}/detachable_joint_state</output_topic>
      </plugin>"""


def payload_geometry_block(legs=False):
    """Visual + collision of the ring payload in the payload link frame: the plate top
    (attach plane) at z = ATTACH_PLANE_Z, ring segments just below it, plate 0 orange.
    Mirrors m2a_ring_fixture.sdf (Te-Jen) shifted so the launches' attach_z stays 0.025.

    The ring and plates are VISUAL ONLY. The single collision is one cylinder the size
    of the ring: 24 box collisions each touching the floor took the RTF from ~30 % to
    7 % (2026-09-23); contact geometry is irrelevant to the carry."""
    out = []
    z_seg = ATTACH_PLANE_Z - PLATE_THICK - RING_SEG_THICK / 2.0
    out.append(f"""          <collision name="collision"><pose>0 0 {_fmt(z_seg)} 0 0 0</pose>
            <geometry><cylinder><radius>{RING_OUTER:.3f}</radius><length>{RING_SEG_THICK:.3f}</length></cylinder></geometry>
          </collision>""")
    for k in range(RING_SEGMENTS):
        th = 2.0 * math.pi * k / RING_SEGMENTS
        pose = (f"{_fmt(PAYLOAD_RADIUS * math.cos(th))} {_fmt(PAYLOAD_RADIUS * math.sin(th))} "
                f"{_fmt(z_seg)} 0 0 {_fmt(th + math.pi / 2.0)}")
        box = (f"<geometry><box><size>{RING_SEG_LEN:.6f} {RING_SEG_RADIAL:.3f} "
               f"{RING_SEG_THICK:.3f}</size></box></geometry>")
        out.append(f"""          <visual name="ring_{k}_visual"><pose>{pose}</pose>{box}
            <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.20 0.20 0.20 1</diffuse></material>
          </visual>""")
    z_plate = ATTACH_PLANE_Z - PLATE_THICK / 2.0
    for k in range(PLATES):
        th = 2.0 * math.pi * k / PLATES
        pose = (f"{_fmt(PAYLOAD_RADIUS * math.cos(th))} {_fmt(PAYLOAD_RADIUS * math.sin(th))} "
                f"{_fmt(z_plate)} 0 0 {_fmt(th)}")
        colour = ("<ambient>0.90 0.25 0.10 1</ambient><diffuse>1.00 0.35 0.15 1</diffuse>" if k == 0
                  else "<ambient>0.65 0.65 0.68 1</ambient><diffuse>0.80 0.80 0.82 1</diffuse>")
        out.append(f"""          <visual name="plate_{k}_visual"><pose>{pose}</pose>
            <geometry><cylinder><radius>{PLATE_RADIUS:.3f}</radius><length>{PLATE_THICK:.3f}</length></cylinder></geometry>
            <material>{colour}</material>
          </visual>""")
    if legs:
        z_leg = ATTACH_PLANE_Z - PLATE_THICK - RING_SEG_THICK - LEG_LEN / 2.0
        for k in range(8):
            th = 2.0 * math.pi * k / 8
            pose = (f"{_fmt(LEG_RING_RADIUS * math.cos(th))} {_fmt(LEG_RING_RADIUS * math.sin(th))} "
                    f"{_fmt(z_leg)} 0 0 0")
            cyl = (f"<geometry><cylinder><radius>{LEG_RADIUS:.3f}</radius>"
                   f"<length>{LEG_LEN:.3f}</length></cylinder></geometry>")
            out.append(f"""          <visual name="leg_{k}_visual"><pose>{pose}</pose>{cyl}
            <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.20 0.20 0.20 1</diffuse></material>
          </visual>""")
    return "\n".join(out)


def payload_rest_z(legs=False):
    """Link-frame z of the payload's lowest point, negated: spawn the payload at this
    height so it rests on the floor without a drop."""
    bottom = ATTACH_PLANE_Z - PLATE_THICK - RING_SEG_THICK - (LEG_LEN if legs else 0.0)
    return -bottom


def nominal_placement(n, cable_len, elev_deg, attach_radius, attach_z, payload_z,
                      azimuths_deg=None):
    """The default world-aligned layout: payload at the origin with zero yaw, drone
    i out along attach azimuth 2*pi*i/n at elev_deg, every drone facing world +x.

    Returned in the same form generate_random_world.py builds, so both feed the
    identical build_world():
        payload     (x, y, z, yaw)
        attaches    [(x, y, z)] world-frame cable attach points
        drones      [(x, y, z)] world-frame drone spawns
        drone_yaws  [rad] per-drone spawn heading
    """
    phi = math.radians(elev_deg)
    horiz = cable_len * math.cos(phi)
    vert = cable_len * math.sin(phi)
    drones, attaches = [], []
    for i in range(n):
        th = (math.radians(azimuths_deg[i]) if azimuths_deg else 2.0 * math.pi * i / n)
        ax = attach_radius * math.cos(th)
        ay = attach_radius * math.sin(th)
        az = payload_z + attach_z
        attaches.append((ax, ay, az))
        drones.append((ax + horiz * math.cos(th), ay + horiz * math.sin(th),
                       az + vert))
    return {'payload': (0.0, 0.0, payload_z, 0.0), 'attaches': attaches,
            'drones': drones, 'drone_yaws': [0.0] * n}


def build(n, cable_len, elev_deg, attach_radius, attach_z, payload_z,
          detachable=False, azimuths_deg=None, legs=False):
    return build_world(n, nominal_placement(n, cable_len, elev_deg, attach_radius,
                                            attach_z, payload_z, azimuths_deg),
                       detachable=detachable, legs=legs)


def build_world(n, placement, detachable=False, legs=False):
    """Emit the world SDF for an explicit placement (see nominal_placement). Every
    tether is a rigid rod of exactly the spawn attach->drone distance, so a
    placement that keeps that distance constant keeps the planner's single
    cable_len valid."""
    px, py, pz, pyaw = placement['payload']
    attaches = placement['attaches']
    drones = placement['drones']
    drone_yaws = placement.get('drone_yaws', [0.0] * n)
    includes = []
    for i in range(n):
        dx, dy, dz = drones[i]
        includes.append(f"""      <include>
        <uri>models/x3_drone{i}.sdf</uri>
        <name>x3_drone{i}</name>
        <pose>{_fmt(dx)} {_fmt(dy)} {_fmt(dz)} 0 0 {_fmt(drone_yaws[i])}</pose>
      </include>""")
    tethers = "".join(tether_block(i, drones[i], attaches[i],
                                   detachable=detachable)
                      for i in range(n))
    # Per-drone DetachableJoint plugins (model-level), only when detachable.
    detaches = ("".join(detachable_joint_block(i) for i in range(n))
                if detachable else "")
    # takeoff platforms: static pillars from the ground to just under each drone
    # so the (stiff, rigid-cabled) drones REST at their design pose until TAKEOFF
    # instead of collapsing to the floor while disarmed.
    platforms = []
    for i, (dx, dy, dz) in enumerate(drones):
        h = dz - 0.1
        # GROUND START: with the drones already on the floor there is nothing to
        # stand them on, and a zero/negative-height box is invalid SDF. Skip.
        if h < 0.05:
            continue
        platforms.append(f"""    <model name="platform_{i}">
      <static>true</static>
      <pose>{_fmt(dx)} {_fmt(dy)} {_fmt(h / 2.0)} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.30 0.30 {h:.6f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.30 0.30 {h:.6f}</size></box></geometry>
          <material><ambient>0.35 0.35 0.4 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material></visual>
      </link>
    </model>""")
    platforms = "\n".join(platforms)
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="quadcopter">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>

    <scene><background>0.6 0.8 1.0 1</background><ambient>0.5 0.5 0.5 1</ambient></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.2 0.5 0.2 1</ambient><diffuse>0.3 0.6 0.3 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>

    <model name="lift_system" canonical_link="anchor">
      <pose>0 0 0 0 0 0</pose>
      <!-- WORLD-FIXED ANCHOR: nested poses publish in the WORLD frame. -->
      <link name="anchor">
        <inertial><mass>0.001</mass>
          <inertia><ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz></inertia>
        </inertial>
      </link>
      <joint name="anchor_to_world" type="fixed"><parent>world</parent><child>anchor</child></joint>

      <!-- ===== DRONES (elevated, taut rigid cables) ===== -->
{chr(10).join(includes)}

      <!-- ===== PAYLOAD: 500 mm ring, 12 magnet plates every 30 deg on the top face, plate 0 on +x (m2a_ring_fixture.sdf) ===== -->
      <model name="payload">
        <pose>{_fmt(px)} {_fmt(py)} {_fmt(pz)} 0 0 {_fmt(pyaw)}</pose>
        <link name="body">
          <gravity>1</gravity>
          <inertial><mass>{PAYLOAD_MASS}</mass>
            <inertia><ixx>{_IXX:.3e}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{_IXX:.3e}</iyy><iyz>0</iyz><izz>{_IZZ:.3e}</izz></inertia>
          </inertial>
{payload_geometry_block(legs)}
        </link>
        <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
          <publish_link_pose>true</publish_link_pose>
          <publish_nested_model_pose>true</publish_nested_model_pose>
          <use_pose_vector_msg>true</use_pose_vector_msg>
          <update_frequency>500</update_frequency>
        </plugin>
      </model>
{tethers}{detaches}
    </model>

    <!-- ===== TAKEOFF PLATFORMS (rest the drones at spawn until TAKEOFF) ===== -->
{platforms}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--cable-len', type=float, default=1.0)
    ap.add_argument('--elev', type=float, default=45.0, help='cable elevation deg')
    ap.add_argument('--attach-radius', type=float, default=PAYLOAD_RADIUS)
    ap.add_argument('--attach-z', type=float, default=0.025)
    ap.add_argument('--payload-z', type=float, default=None,
                    help='payload link z at spawn (default 0.025, the historical disc-centre height; '
                         'the ring settles 1.5 cm onto the floor at t=0. With --legs: resting on the legs)')
    ap.add_argument('--azimuths', type=str, default='',
                    help="attach azimuths deg, e.g. '0,90,180' (3/12/9 o'clock); '' = even")
    ap.add_argument('--out', type=str, default='three_rigid.sdf')
    ap.add_argument('--detachable', action='store_true',
                    help='make each cable releasable at the PAYLOAD end, keyed to '
                         '/drone_k/detach, so a drone flies away WITH its cable mid-flight '
                         '(payload left clean). Used by the dissipative detach controller.')
    ap.add_argument('--payload-mass', type=float, default=None,
                    help='payload mass kg (default: the module PAYLOAD_MASS, 0.86); the ring '
                         'inertia scales with it. Match the launch load_mass.')
    ap.add_argument('--legs', action='store_true',
                    help="add the fixture's 8 bench legs (bench picture only; the flying "
                         'payload rests on its ring)')
    ap.add_argument('--ground-start', action='store_true',
                    help='place the drones ON THE FLOOR: overrides --elev so the '
                         'rod runs from the payload attach point out to a drone '
                         'at ground height (a near-horizontal rod). Use with the '
                         "planner's handover_elev_deg so it creeps up to a "
                         'liftable angle before taking over.')
    a = ap.parse_args()
    if a.payload_mass is not None:
        global PAYLOAD_MASS, _IXX, _IZZ
        PAYLOAD_MASS = float(a.payload_mass)
        _IXX, _IZZ = _inertia(PAYLOAD_MASS)
    if a.payload_z is None:
        a.payload_z = payload_rest_z(True) if a.legs else 0.025
    elev = a.elev
    if a.ground_start:
        # drone_z = payload_z + attach_z + cable_len*sin(elev); solve for the
        # elev that puts the drone at DRONE_GROUND_Z.
        DRONE_GROUND_Z = 0.10
        need = (DRONE_GROUND_Z - a.payload_z - a.attach_z) / a.cable_len
        elev = math.degrees(math.asin(max(-1.0, min(1.0, need))))
        print(f'ground start: elev overridden to {elev:.2f} deg '
              f'(drone z = {DRONE_GROUND_Z}, payload z = {a.payload_z})')
    az = [float(v) for v in a.azimuths.split(',') if v.strip()] or None
    if az and len(az) != a.n:
        ap.error(f'--azimuths has {len(az)} entries for --n {a.n}')
    sdf = build(a.n, a.cable_len, elev, a.attach_radius, a.attach_z, a.payload_z,
                detachable=a.detachable, azimuths_deg=az, legs=a.legs)
    with open(a.out, 'w') as f:
        f.write(sdf)
    print(f"wrote {a.out}: n={a.n} cable_len={a.cable_len} elev={a.elev}deg "
          f"detachable={a.detachable}")


if __name__ == '__main__':
    main()
