#!/usr/bin/env python3
"""Bridge raw Gazebo detachable-joint *detached* state into a ROS Bool.

The raw Gazebo state is validation ground truth only. ``True`` means detached.
The latest state is periodically republished so observers started just after the
paused bootstrap do not miss the one-shot Gazebo state transition.
"""

from __future__ import annotations

import subprocess
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from tejen_mission.m2a_attachment_runtime import parse_gz_detachable_joint_state_line


class GazeboJointTruthBridge(Node):
    def __init__(self):
        super().__init__("m2a_gazebo_joint_truth_bridge")
        self.gz_topic = str(
            self.declare_parameter("gz_topic", "/payload/detachable_joint_state").value
        )
        self.ros_topic = str(
            self.declare_parameter("ros_topic", "/m2a/sim/joint_detached_truth").value
        )
        self.republish_period_s = max(
            0.02, float(self.declare_parameter("republish_period_s", 1.0).value)
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "m2a/sim/joint_truth_bridge_state"
            ).value
        )
        self.pub = self.create_publisher(Bool, self.ros_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.process: Optional[subprocess.Popen] = None
        self.thread: Optional[threading.Thread] = None
        self.last_detached: Optional[bool] = None
        self._start_process()
        self.timer = self.create_timer(self.republish_period_s, self._health)

    def _start_process(self):
        if self.process is not None and self.process.poll() is None:
            return
        self.process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.gz_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _publish_truth(self, detached: bool):
        self.last_detached = bool(detached)
        msg = Bool()
        msg.data = self.last_detached
        self.pub.publish(msg)

    def _reader(self):
        assert self.process is not None and self.process.stdout is not None
        for line in self.process.stdout:
            detached = parse_gz_detachable_joint_state_line(line)
            if detached is None:
                continue
            self._publish_truth(detached)

    def _health(self):
        status = String()
        if self.process is None or self.process.poll() is not None:
            status.data = "restarting"
            self.status_pub.publish(status)
            self._start_process()
        else:
            status.data = "running_raw_detached_state"
            self.status_pub.publish(status)
        if self.last_detached is not None:
            self._publish_truth(self.last_detached)

    def destroy_node(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboJointTruthBridge()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
