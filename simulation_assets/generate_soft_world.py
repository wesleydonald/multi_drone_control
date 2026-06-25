#!/usr/bin/env python3
"""
generate_soft_world.py
----------------------
Generate an N-drone soft-cable lift world (matching the structure/conventions of
the hand-made two_soft.sdf / four_soft.sdf):

  - world `quadcopter` (name kept so existing /world/quadcopter/* topics + the
    mocap world-pose source still apply)
  - lift_system model with the world-fixed `anchor` canonical link (so nested
    poses publish in the WORLD frame — see two_no_cables.sdf)
  - N drones (models/x3_drone{i}.sdf) at equal angular intervals
  - rigid-body payload box with N DISTINCT attachment points (radius R_a) so a
    team of N>=3 can control the full 6-DOF load pose (Sun et al. 2025)
  - N soft cables, each `n_seg` cylinder segments joined by universal joints,
    chaining payload::body -> seg(last) -> ... -> seg0 -> droneN::base_link

Cable geometry: each segment lies on the straight line from the drone to its
payload attachment point; the cylinder local-z axis is aligned with the cable
direction via  pitch = acos(u_z), yaw = atan2(u_y, u_x).

Usage:
    python3 generate_soft_world.py --n 3 --out three_soft.sdf
    python3 generate_soft_world.py --n 4 --out four_soft_gen.sdf
"""
import argparse
import math


def _solve_arc_angle(k):
    """Solve alpha/sin(alpha) = k for alpha in (0, pi) by bisection (k > 1)."""
    lo, hi = 1e-6, math.pi - 1e-6
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        # f(alpha) = alpha - k*sin(alpha); increasing through the root on (0, pi)
        if mid - k * math.sin(mid) < 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def cable_block(idx, drone_xyz, attach_xyz, n_seg, cable_len=None,
                seg_mass=0.01, radius=0.005):
    """Return SDF text (links + joints) for one cable from a drone to the load.

    If `cable_len` is given and exceeds the straight drone->attach distance, the
    cable is SLACK: its kinematic length is `cable_len` and the segments are laid
    out along a downward-sagging circular arc (so the universal-joint anchors
    coincide at spawn — no snap). A slack cable lets the drone climb ABOVE the
    load and develop vertical lifting tension; a taut cable (len == distance)
    pins the drone to a near-horizontal shell and cannot lift. See the geometry
    discussion in the launch notes.
    """
    dx = attach_xyz[0] - drone_xyz[0]
    dy = attach_xyz[1] - drone_xyz[1]
    dz = attach_xyz[2] - drone_xyz[2]
    chord = math.sqrt(dx * dx + dy * dy + dz * dz)

    slack = cable_len is not None and cable_len > chord * 1.001
    L = cable_len if slack else chord
    seg_len = L / n_seg
    half = seg_len / 2.0

    # cylinder inertia (thin rod about transverse axes ~ m*l^2/12)
    it = seg_mass * seg_len * seg_len / 12.0
    ia = 0.5 * seg_mass * radius * radius

    # per-segment center positions + orientations (cylinder local-z = cable dir)
    centers, pitches, yaws = [], [], []
    if not slack:
        ux, uy, uz = dx / chord, dy / chord, dz / chord
        pitch = math.acos(max(-1.0, min(1.0, uz)))
        yaw = math.atan2(uy, ux)
        for j in range(n_seg):
            frac = (j + 0.5) / n_seg
            centers.append((drone_xyz[0] + frac * dx,
                            drone_xyz[1] + frac * dy,
                            drone_xyz[2] + frac * dz))
            pitches.append(pitch)
            yaws.append(yaw)
    else:
        # downward-sagging circular arc: chord = `chord`, arc length = L.
        alpha = _solve_arc_angle(L / chord)          # half subtended angle
        r_arc = chord / (2.0 * math.sin(alpha))
        tx, ty, tz = dx / chord, dy / chord, dz / chord            # chord unit
        # n = component of "down" perpendicular to the chord (sag direction)
        d_dot = -tz                                               # down=(0,0,-1)
        nx, ny, nz = -d_dot * tx, -d_dot * ty, -1.0 - d_dot * tz
        nn = math.sqrt(nx * nx + ny * ny + nz * nz)
        nx, ny, nz = nx / nn, ny / nn, nz / nn
        mx = 0.5 * (drone_xyz[0] + attach_xyz[0])
        my = 0.5 * (drone_xyz[1] + attach_xyz[1])
        mz = 0.5 * (drone_xyz[2] + attach_xyz[2])
        a = r_arc * math.cos(alpha)
        cx0, cy0, cz0 = mx - nx * a, my - ny * a, mz - nz * a     # arc center
        for j in range(n_seg):
            phi = -alpha + (j + 0.5) * (2.0 * alpha / n_seg)
            cph, sph = math.cos(phi), math.sin(phi)
            centers.append((cx0 + r_arc * (cph * nx + sph * tx),
                            cy0 + r_arc * (cph * ny + sph * ty),
                            cz0 + r_arc * (cph * nz + sph * tz)))
            # unit tangent (drone -> payload direction along the arc)
            ttx = -sph * nx + cph * tx
            tty = -sph * ny + cph * ty
            ttz = -sph * nz + cph * tz
            pitches.append(math.acos(max(-1.0, min(1.0, ttz))))
            yaws.append(math.atan2(tty, ttx))

    s = [f"      <!-- ===== CABLE {idx}: drone{idx} -> payload "
         f"(len={L:.4f}, seg_len={seg_len:.4f}, "
         f"{'SLACK' if slack else 'taut'}) ===== -->"]

    # seg0 is at the drone end, seg(n_seg-1) at the payload end
    for j in range(n_seg):
        cx, cy, cz = centers[j]
        s.append(f"""      <link name="cable{idx}_seg{j}">
        <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 {pitches[j]:.6f} {yaws[j]:.6f}</pose>
        <inertial>
          <mass>{seg_mass}</mass>
          <inertia>
            <ixx>{it:.3e}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{it:.3e}</iyy><iyz>0</iyz><izz>{ia:.3e}</izz>
          </inertia>
        </inertial>
        <visual name="v">
          <geometry><cylinder><radius>{radius}</radius><length>{seg_len:.6f}</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
      </link>""")

    last = n_seg - 1

    def joint(name, parent, child, off):
        return f"""      <joint name="{name}" type="universal">
        <parent>{parent}</parent>
        <child>{child}</child>
        <pose relative_to="{child}">0 0 {off:.6f} 0 0 0</pose>
        <axis><xyz>1 0 0</xyz><dynamics><damping>0.01</damping></dynamics></axis>
        <axis2><xyz>0 1 0</xyz><dynamics><damping>0.01</damping></dynamics></axis2>
      </joint>"""

    # payload end: payload::body -> seg(last) at its +z (payload-facing) end
    s.append(joint(f"cable{idx}_payload", "payload::body", f"cable{idx}_seg{last}", +half))
    # chain: seg(k+1) -> seg(k) at seg(k)'s +z end
    for k in range(last):
        s.append(joint(f"cable{idx}_j{k}", f"cable{idx}_seg{k+1}", f"cable{idx}_seg{k}", +half))
    # drone end: seg0 -> drone base_link at seg0's -z (drone-facing) end
    s.append(joint(f"cable{idx}_drone", f"cable{idx}_seg0", f"x3_drone{idx}::base_link", -half))
    return "\n".join(s)


def generate(n, R_d=0.35, z_d=0.1, R_a=0.08, payload_z=0.025, payload_top=0.05,
             n_seg=5, payload_mass=0.3, cable_len=None):
    drones, attaches, includes = [], [], []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        d = (R_d * math.cos(th), R_d * math.sin(th), z_d)
        a = (R_a * math.cos(th), R_a * math.sin(th), payload_top)
        drones.append(d)
        attaches.append(a)
        includes.append(f"""      <include>
        <uri>models/x3_drone{i}.sdf</uri>
        <name>x3_drone{i}</name>
        <pose>{d[0]:.6f} {d[1]:.6f} {d[2]:.6f} 0 0 0</pose>
      </include>""")

    cables = "\n".join(cable_block(i, drones[i], attaches[i], n_seg, cable_len)
                       for i in range(n))
    # payload box inertia (0.2 x 0.2 x 0.05)
    bx, by, bz = 0.2, 0.2, 0.05
    ixx = payload_mass * (by * by + bz * bz) / 12.0
    iyy = payload_mass * (bx * bx + bz * bz) / 12.0
    izz = payload_mass * (bx * bx + by * by) / 12.0

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="quadcopter">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>

    <scene>
      <background>0.6 0.8 1.0 1</background>
      <ambient>0.5 0.5 0.5 1</ambient>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.2 0.5 0.2 1</ambient><diffuse>0.3 0.6 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="lift_system" canonical_link="anchor">
      <pose>0 0 0 0 0 0</pose>

      <!-- WORLD-FIXED ANCHOR: pins the lift_system frame to the world origin so
           nested drone/payload poses are published in the WORLD frame. -->
      <link name="anchor">
        <inertial>
          <mass>0.001</mass>
          <inertia>
            <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="anchor_to_world" type="fixed">
        <parent>world</parent>
        <child>anchor</child>
      </joint>

      <!-- ===== DRONES (equal angular intervals, resting near ground) ===== -->
{chr(10).join(includes)}

      <!-- ===== PAYLOAD (rigid body, {n} distinct attachment points at R_a={R_a}) ===== -->
      <model name="payload">
        <pose>0 0 {payload_z} 0 0 0</pose>
        <link name="body">
          <gravity>1</gravity>
          <inertial>
            <mass>{payload_mass}</mass>
            <inertia>
              <ixx>{ixx:.3e}</ixx><ixy>0</ixy><ixz>0</ixz>
              <iyy>{iyy:.3e}</iyy><iyz>0</iyz><izz>{izz:.3e}</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry><box><size>{bx} {by} {bz}</size></box></geometry>
          </collision>
          <visual name="visual">
            <geometry><box><size>{bx} {by} {bz}</size></box></geometry>
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

{cables}
    </model>
  </world>
</sdf>
"""


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3, help="number of drones")
    ap.add_argument("--out", type=str, default="three_soft.sdf")
    ap.add_argument("--seg", type=int, default=5, help="segments per cable")
    ap.add_argument("--payload-mass", type=float, default=0.3)
    ap.add_argument("--r-drone", type=float, default=0.5,
                    help="horizontal distance from payload center to each drone (m)")
    ap.add_argument("--cable-len", type=float, default=None,
                    help="cable length (m). If > the drone->attach distance the "
                         "cable is SLACK (drones can climb above the load and "
                         "lift it). Default: taut at spawn (legacy).")
    args = ap.parse_args()
    sdf = generate(args.n, R_d=args.r_drone, n_seg=args.seg,
                   payload_mass=args.payload_mass, cable_len=args.cable_len)
    with open(args.out, "w") as f:
        f.write(sdf)
    print(f"Wrote {args.out}  ({args.n} drones, {args.seg} segments/cable)")
