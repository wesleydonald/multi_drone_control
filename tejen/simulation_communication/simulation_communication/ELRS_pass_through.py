


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry  # Import the Telemetry message


class ELRSPassThrough(Node):
    def __init__(self):
        super().__init__('ELRS_pass_through')
        self.publisher = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)
        self.subscription_floats = self.create_subscription(ELRSCommand, 'ELRSCommand', self.controller_commands_callback, 10)



    def controller_commands_callback(self, msg):

        max_rot_val = 4631.0
        self.speed = [msg.channel_0*max_rot_val,msg.channel_1*max_rot_val,msg.channel_2*max_rot_val,msg.channel_3*max_rot_val,msg.channel_4*max_rot_val,msg.channel_5*max_rot_val,msg.channel_6*max_rot_val,msg.channel_7*max_rot_val]
        actuator_msg = Actuators()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.velocity = self.speed
        self.publisher.publish(actuator_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ELRSPassThrough()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
