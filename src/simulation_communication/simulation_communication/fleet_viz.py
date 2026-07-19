"""
fleet_viz.py
------------
Visualization-only node: turns mocap state into things RViz can draw, for the
drones AND the payload, WITHOUT any controller running.

Why this exists: the TF frames the drone meshes attach to (map -> drone_i_mocap)
used to be broadcast by each tracker's TrajectoryVisualizer. That meant nothing
appeared in RViz until the controllers were up, so you could not bring up RViz
first and confirm the fleet was present and publishing before committing to a
flight. This node owns those frames instead, so the viz stack stands alone.
(The tracker no longer broadcasts them — two publishers of the same frame just
produces TF_REPEATED_DATA warnings.)

Publishes:
    TF  map -> drone_i_mocap     one per drone, from /drone_i/motion_capture_state
    TF  map -> payload_mocap     from /payload/motion_capture_state
    /payload/marker              the payload box (matches the world SDF)
    /payload/actual_path         where the payload has actually been

Params:
    num_drones      (int)   fleet size
    payload_size    (float[3]) payload box x,y,z in metres (world SDF: 0.2 0.2 0.05)
    path_max_len    (int)   ring-buffer length for the payload track
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

from interfaces.msg import MotionCaptureState

FRAME = 'map'


class FleetViz(Node):
    def __init__(self):
        super().__init__('fleet_viz')
        self.n = int(self.declare_parameter('num_drones', 3).value)
        self.payload_size = [float(v) for v in self.declare_parameter(
            'payload_size', [0.2, 0.2, 0.05]).value]
        self.path_max_len = int(self.declare_parameter('path_max_len', 2000).value)

        self.tf = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/payload/marker', 1)
        self.path_pub = self.create_publisher(Path, '/payload/actual_path', 5)
        self._payload_path = []

        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda m, k=i: self._drone_cb(m, k), 5)
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)

        self.get_logger().info(
            f'fleet_viz up: broadcasting TF for {self.n} drones + payload, '
            f'payload box {self.payload_size}')

    def _send_tf(self, msg, child):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME
        t.child_frame_id = child
        p, o = msg.pose.position, msg.pose.orientation
        t.transform.translation.x = float(p.x)
        t.transform.translation.y = float(p.y)
        t.transform.translation.z = float(p.z)
        t.transform.rotation.w = float(o.w)
        t.transform.rotation.x = float(o.x)
        t.transform.rotation.y = float(o.y)
        t.transform.rotation.z = float(o.z)
        self.tf.sendTransform(t)

    def _drone_cb(self, msg, i):
        self._send_tf(msg, f'drone_{i}_mocap')

    def _payload_cb(self, msg):
        self._send_tf(msg, 'payload_mocap')

        # box marker, anchored to the payload frame so it moves with it
        m = Marker()
        m.header.frame_id = 'payload_mocap'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'payload'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = self.payload_size
        m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.4, 0.0, 0.9
        self.marker_pub.publish(m)

        # actual track
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = m.header.stamp
        ps.pose = msg.pose
        self._payload_path.append(ps)
        if len(self._payload_path) > self.path_max_len:
            self._payload_path = self._payload_path[-self.path_max_len:]
        path = Path()
        path.header.frame_id = FRAME
        path.header.stamp = m.header.stamp
        path.poses = self._payload_path
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = FleetViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
