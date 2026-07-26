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
  - rigid-body payload box with the pose publisher
  - N rigid tethers (one cylinder link each), ball joint to payload::body at the
    bottom and to x3_drone{i}::base_link at the top

Usage:
    python3 generate_rigid_world.py --n 3 --out three_rigid.sdf
"""
import argparse
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


def tether_block(idx, drone, attach, cable_len, radius=0.004, mass=0.001,
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


def build(n, cable_len, elev_deg, attach_radius, attach_z, payload_z,
          detachable=False):
    phi = math.radians(elev_deg)
    horiz = cable_len * math.cos(phi)
    vert = cable_len * math.sin(phi)
    drones, attaches, includes = [], [], []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        ax = attach_radius * math.cos(th)
        ay = attach_radius * math.sin(th)
        az = payload_z + attach_z
        dx = ax + horiz * math.cos(th)
        dy = ay + horiz * math.sin(th)
        dz = az + vert
        attaches.append((ax, ay, az))
        drones.append((dx, dy, dz))
        includes.append(f"""      <include>
        <uri>models/x3_drone{i}.sdf</uri>
        <name>x3_drone{i}</name>
        <pose>{_fmt(dx)} {_fmt(dy)} {_fmt(dz)} 0 0 0</pose>
      </include>""")
    tethers = "".join(tether_block(i, drones[i], attaches[i], cable_len,
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

      <!-- ===== PAYLOAD ===== -->
      <model name="payload">
        <pose>0 0 {payload_z} 0 0 0</pose>
        <link name="body">
          <gravity>1</gravity>
          <inertial><mass>0.4</mass>
            <inertia><ixx>1.67e-03</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1.67e-03</iyy><iyz>0</iyz><izz>3.33e-03</izz></inertia>
          </inertial>
          <collision name="collision"><geometry><box><size>0.2 0.2 0.05</size></box></geometry></collision>
          <visual name="visual"><geometry><box><size>0.2 0.2 0.05</size></box></geometry>
            <material><ambient>0.8 0.4 0.0 1</ambient><diffuse>0.8 0.4 0.0 1</diffuse></material>
          </visual>
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
    ap.add_argument('--attach-radius', type=float, default=0.08)
    ap.add_argument('--attach-z', type=float, default=0.025)
    ap.add_argument('--payload-z', type=float, default=0.025)
    ap.add_argument('--out', type=str, default='three_rigid.sdf')
    ap.add_argument('--detachable', action='store_true',
                    help='make each cable releasable at the PAYLOAD end, keyed to '
                         '/drone_k/detach, so a drone flies away WITH its cable mid-flight '
                         '(payload left clean). Used by the dissipative detach controller.')
    ap.add_argument('--ground-start', action='store_true',
                    help='place the drones ON THE FLOOR: overrides --elev so the '
                         'rod runs from the payload attach point out to a drone '
                         'at ground height (a near-horizontal rod). Use with the '
                         "planner's handover_elev_deg so it creeps up to a "
                         'liftable angle before taking over.')
    a = ap.parse_args()
    elev = a.elev
    if a.ground_start:
        # drone_z = payload_z + attach_z + cable_len*sin(elev); solve for the
        # elev that puts the drone at DRONE_GROUND_Z.
        DRONE_GROUND_Z = 0.10
        need = (DRONE_GROUND_Z - a.payload_z - a.attach_z) / a.cable_len
        elev = math.degrees(math.asin(max(-1.0, min(1.0, need))))
        print(f'ground start: elev overridden to {elev:.2f} deg '
              f'(drone z = {DRONE_GROUND_Z}, payload z = {a.payload_z})')
    sdf = build(a.n, a.cable_len, elev, a.attach_radius, a.attach_z, a.payload_z,
                detachable=a.detachable)
    with open(a.out, 'w') as f:
        f.write(sdf)
    print(f"wrote {a.out}: n={a.n} cable_len={a.cable_len} elev={a.elev}deg "
          f"detachable={a.detachable}")


if __name__ == '__main__':
    main()
