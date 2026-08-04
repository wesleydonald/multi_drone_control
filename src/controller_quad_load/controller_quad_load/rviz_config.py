"""
rviz_config.py — the ONE RViz layout, generated for any fleet size (finding F8).

Both `rviz_quad_load_launch.py` (simulation) and `real_io_launch.py` (hardware) build
their config from here, so what you rehearse in sim is what you get at the rig.

WHY THIS EXISTS
---------------
There used to be two near-identical `_build_config` functions, one per launch file, and
they had already drifted: the hardware one had **no DETACH/ATTACH panel buttons**, no
drone-ID labels and no per-drone reference display. So the display you would be staring
at during an actual flight test was the less capable one, and nobody would notice until
they were standing in the lab wondering where the DETACH button went.

The differences that are real are parameters, not forks:

  detach / attach   whether those panel buttons are shown (they drive the dissipative
                    and magnet stacks, so a launch that does not run them hides them)
  show_actual       flown-path trails; noisy in sim, useful at the rig
  show_plan         per-drone MPC plan (only meaningful when a tracker is running)

Everything else — colours, payload, drone models, panel, view — is shared by
construction. A new display added here appears in both.

COLOURS are keyed by drone id and are the SAME palette the analysis plots use, so
drone 2 is the same colour in RViz and in every thesis figure.
"""

# per-drone colour: (rviz "r; g; b" for paths, urdf "r g b a" for the mesh)
DRONE_COLOURS = [
    ('31; 119; 180',  '0.12 0.47 0.71 0.9'),   # blue
    ('255; 127; 14',  '1.00 0.50 0.05 0.9'),   # orange
    ('44; 160; 44',   '0.17 0.63 0.17 0.9'),   # green
    ('214; 39; 40',   '0.84 0.15 0.16 0.9'),   # red
    ('148; 103; 189', '0.58 0.40 0.74 0.9'),   # purple
]
PAYLOAD_COLOUR = '255; 215; 0'    # gold


def drone_colour(i):
    """RViz colour string for drone `i`. Wraps for fleets larger than the palette."""
    return DRONE_COLOURS[i % len(DRONE_COLOURS)][0]


def drone_mesh_colour(i):
    """URDF rgba string for drone `i`."""
    return DRONE_COLOURS[i % len(DRONE_COLOURS)][1]


def _path_display(name, topic, colour, width, enabled=True):
    return f"""    - Class: rviz_default_plugins/Path
      Name: {name}
      Enabled: {str(enabled).lower()}
      Topic:
        Value: {topic}
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Color: {colour}
      Line Style: Lines
      Line Width: {width}
      Alpha: 1
      Buffer Length: 1
      Offset: {{X: 0, Y: 0, Z: 0}}
      Pose Style: None"""


def _robot_display(i, enabled=True):
    return f"""    - Class: rviz_default_plugins/RobotModel
      Name: Drone {i} airframe
      Enabled: {str(enabled).lower()}
      Description Source: Topic
      Description Topic:
        Value: /robot_description_{i}
        Depth: 5
        Durability Policy: Transient Local
        Reliability Policy: Reliable
      Description File: ""
      TF Prefix: ""
      Alpha: 1
      Visual Enabled: true
      Collision Enabled: false
      Update Interval: 0"""


def _marker_array(name, topic, namespace):
    return f"""    - Class: rviz_default_plugins/MarkerArray
      Name: {name}
      Enabled: true
      Topic:
        Value: {topic}
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Namespaces:
        {namespace}: true"""


def build_config(n: int, *, show_actual: bool = False, detach: bool = False,
                 attach: bool = False, show_plan: bool = True,
                 width: int = 1400, height: int = 900) -> str:
    """Return a complete .rviz config for exactly `n` drones.

    Identical in sim and on hardware except for the flags above, which reflect what
    each launch actually runs rather than which file you happened to open."""
    displays = ["""    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: 20
      Color: 160; 160; 164
      Alpha: 0.5
      Line Style:
        Line Width: 0.03
        Value: Lines
      Plane: XY
      Normal Cell Count: 0
      Offset: {X: 0, Y: 0, Z: 0}
      Reference Frame: <Fixed Frame>""",
                """    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
      Show Names: true
      Show Axes: true
      Show Arrows: false
      Marker Scale: 0.3
      Update Interval: 0
      Frame Timeout: 15"""]

    # payload first so it draws under the drones
    displays.append("""    - Class: rviz_default_plugins/Marker
      Name: Payload box
      Enabled: true
      Topic:
        Value: /payload/marker
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Namespaces:
        payload: true""")
    displays.append(_path_display(
        'Payload desired', '/payload/mpc_plan', PAYLOAD_COLOUR, 0.03))
    displays.append(_path_display(
        'Payload actual', '/payload/actual_path', '255; 255; 255', 0.015,
        enabled=show_actual))

    # floating drone-id labels (fleet_viz /fleet/id_markers), one text per drone.
    # Previously sim-only; on hardware it is arguably MORE useful, because you
    # cannot tell the airframes apart by eye across the cage.
    displays.append(_marker_array('Drone IDs', '/fleet/id_markers', 'drone_id'))

    for i in range(n):
        c = drone_colour(i)
        displays.append(_robot_display(i))
        if show_plan:
            displays.append(_path_display(
                f'Drone {i} MPC plan', f'/drone_{i}/mpc_plan', c, 0.02))
        displays.append(_path_display(
            f'Drone {i} actual', f'/drone_{i}/actual_path', c, 0.01,
            enabled=show_actual))
        displays.append(_path_display(
            f'Drone {i} reference', f'/drone_{i}/trajectory_path', c, 0.01,
            enabled=False))

    body = "\n".join(displays)
    return f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded: ~
    Tree Height: 500
  - Class: drone_visualisation/ArmPanel
    Name: ArmPanel
    ShowDetach: {str(detach).lower()}
    ShowAttach: {str(attach).lower()}
    NumDrones: {n}
Visualization Manager:
  Class: ""
  Name: root
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
    Frame Rate: 30
  Displays:
{body}
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Distance: 4
      Focal Point: {{X: 0, Y: 0, Z: 0.5}}
      Pitch: 0.4
      Yaw: 0.8
      Target Frame: <Fixed Frame>
      Near Clip Distance: 0.01
Window Geometry:
  Height: {height}
  Width: {width}
  Displays:
    collapsed: false
  ArmPanel:
    collapsed: false
"""
# NB `Window Geometry:` is NOT optional and must stay at column 0. Without it,
# Height/Width/Displays/ArmPanel fall inside `Visualization Manager:`, which already
# has a `Displays:` key -- YAML takes the LAST duplicate, so `Displays: {collapsed:
# false}` silently replaces the entire display list and RViz opens with nothing in
# it. The launch looks completely healthy while showing an empty scene. This was
# broken for exactly one commit on 2026-08-04; do not "tidy" it away.
