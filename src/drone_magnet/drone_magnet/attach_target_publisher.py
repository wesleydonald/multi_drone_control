"""
attach_target_publisher.py
--------------------------
Feeds the online_join_planner its attachment target for THIS world. The planner (built for
a different sim) expects the magnet-tip target as a PoseStamped + TwistStamped on dedicated
topics; here the target is simply the shared lift_system payload -- the body the magnet welds
to. This node republishes the payload's mocap (/payload/motion_capture_state, world frame) as:

    <pose_topic>   geometry_msgs/PoseStamped   (payload pose, + z_offset for the attach face)
    <twist_topic>  geometry_msgs/TwistStamped  (payload velocity, so the planner tracks a
                                                 lifting/moving load)

The planner adds its own magnet_drop_below_quad (the cable length) to turn this magnet-tip
target into a quad-body reference, so point this at where the MAGNET TIP should end up (the
payload body), not where the drone should fly.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from interfaces.msg import MotionCaptureState


class AttachTargetPublisher(Node):
    def __init__(self):
        super().__init__('attach_target_publisher')
        payload_topic = self.declare_parameter(
            'payload_state_topic', '/payload/motion_capture_state').value
        self.pose_topic = self.declare_parameter('pose_topic', '/attach_target/pose').value
        self.twist_topic = self.declare_parameter('twist_topic', '/attach_target/twist').value
        # lift the target above the payload centre so the magnet aims at the top face.
        self.z_offset = float(self.declare_parameter('z_offset', 0.05).value)
        # OFF-CENTRE weld (world frame). Aiming the magnet tip at the payload CENTRE makes the
        # newcomer a central lifter -- a centre-welded cable can only add vertical lift, never
        # bear load off to a side, so the fleet cannot reconfigure. Shifting the target a small
        # XY offset welds the tip at an off-centre RING point (moment arm != 0), so with the
        # dissipative network's unequal (moment-balanced) force sharing the 4th becomes a real
        # load-bearing member and the fleet visibly redistributes while the load stays level.
        # Keep |offset| within the manager's attach_radius (default 0.15) or it won't weld.
        self.x_offset = float(self.declare_parameter('x_offset', 0.0).value)
        self.y_offset = float(self.declare_parameter('y_offset', 0.0).value)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.create_subscription(
            MotionCaptureState, payload_topic, self._cb, 10)
        self.get_logger().info(
            f'[attach_target] {payload_topic} -> {self.pose_topic} '
            f'(offset x{self.x_offset:+.2f} y{self.y_offset:+.2f} z+{self.z_offset:.2f}) '
            f'+ {self.twist_topic}')

    def _cb(self, msg: MotionCaptureState):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose
        ps.pose.position.x += self.x_offset
        ps.pose.position.y += self.y_offset
        ps.pose.position.z += self.z_offset
        self.pose_pub.publish(ps)

        ts = TwistStamped()
        ts.header = msg.header
        ts.twist = msg.twist
        self.twist_pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = AttachTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
