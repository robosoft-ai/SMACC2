#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rosgraph_msgs.msg

tfbrod = None
clock_msg = None


class StaticTransformPublisher:
    def __init__(self, node, parent_frame, child_frame, xyz=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]):
        self.node = node
        global tfbrod
        print("Initializing static transform publisher")

        self.transform_broadcaster = tfbrod
        print("xyz: ", xyz)

        try:
            self.transform = TransformStamped()
            self.transform.header.frame_id = parent_frame
            self.transform.child_frame_id = child_frame
            self.transform.transform.translation.x = xyz[0]
            self.transform.transform.translation.y = xyz[1]
            self.transform.transform.translation.z = xyz[2]

            self.transform.transform.rotation.x = 0.0
            self.transform.transform.rotation.y = 0.0
            self.transform.transform.rotation.z = 0.0
            self.transform.transform.rotation.w = 1.0
        except Exception as e:
            print(e)
            self.node.get_logger().error("Exception in StaticTransformPublisher: %r" % e)

        self.timer = node.create_timer(0.05, self.publish_transform)

    def publish_transform(self):
        global clock_msg
        if clock_msg is None:
            return

        time = rclpy.time.Time(seconds=clock_msg.clock.sec, nanoseconds=clock_msg.clock.nanosec)
        # time = time + rclpy.time.Duration(seconds=0.01)
        self.transform.header.stamp = time.to_msg()
        self.transform_broadcaster.sendTransform(self.transform)
        self.node.get_logger().info(
            "Publishing transform from %s to %s"
            % (self.transform.header.frame_id, self.transform.child_frame_id)
        )


def main(args=None):
    print("Initializing static transform publisher")
    rclpy.init()

    node = rclpy.create_node("static_transform_publisher_1")

    global tfbrod

    tfbrod = tf2_ros.TransformBroadcaster(node)

    def callback(msg):
        global clock_msg
        clock_msg = msg

    clock_sub = node.create_subscription(
        rosgraph_msgs.msg.Clock,
        "/clock",
        callback,
        qos_profile=rclpy.qos.qos_profile_sensor_data,
    )

    print("Creating static transform publisher nodes")
    # Create the first instance of StaticTransformPublisher
    node1 = StaticTransformPublisher(
        node,
        "base_footprint",
        "base_link",
    )

    #
    # Create the second instance of StaticTransformPublisher
    node2 = StaticTransformPublisher(
        node,
        "base_link",
        "base_scan",
        xyz=[-0.064, 0.0, 0.122],
    )

    node3 = StaticTransformPublisher(
        node,
        "map",
        "odom",
        xyz=[0.0, 0.0, 0.0],
    )

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    import sys

    print("Starting static transform publisher")

    main()
