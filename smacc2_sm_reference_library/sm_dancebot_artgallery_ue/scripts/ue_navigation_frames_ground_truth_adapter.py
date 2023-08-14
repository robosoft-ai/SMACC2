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
import ue_msgs.msg
import nav_msgs.msg


class UENavigationFramesGroundTruthAdapter:
    def __init__(self, parent_frame, child_frame):
        self.node = rclpy.create_node("static_transform_publisher_1")
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.clock_msg = None

        self.parent_frame = parent_frame
        self.child_frame = child_frame

        print("Initializing static transform publisher")

        self.uemsgs_sub = self.node.create_subscription(
            ue_msgs.msg.EntityState,
            "/ue_ros/model_state",
            self.uemsgs_callback,
            # reliability reliable
            rclpy.qos.qos_profile_services_default,
        )

        self.odom_pub = self.node.create_publisher(
            nav_msgs.msg.Odometry, "/odom", rclpy.qos.qos_profile_services_default
        )

        self.clock_sub = self.node.create_subscription(
            rosgraph_msgs.msg.Clock,
            "/clock",
            self.clock_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

    def clock_callback(self, msg):
        self.clock_msg = msg

    def uemsgs_callback(self, msg):

        if self.clock_msg is None:
            return

        t = TransformStamped()
        t.header.stamp = self.clock_msg.clock
        t.header.frame_id = self.parent_frame

        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # self.node.get_logger().info("uemsgs_callback" + str(t))

        self.transform_broadcaster.sendTransform(t)

        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = self.clock_msg.clock
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose = msg.pose

        # set covariance diagonal to 0.1
        odom.pose.covariance = [0.1 if i % 7 == 0 else 0.0 for i in range(36)]

        odom.twist.twist = msg.twist
        odom.twist.covariance = [0.1 if i % 7 == 0 else 0.0 for i in range(36)]

        self.odom_pub.publish(odom)

    def spin(self):
        rclpy.spin(self.node)


def main(args=None):
    print("Initializing static transform publisher")
    rclpy.init()

    ue_navigation_frames_ground_truth_adapter = UENavigationFramesGroundTruthAdapter(
        "odom", "base_footprint"
    )
    ue_navigation_frames_ground_truth_adapter.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys

    print("Starting static transform publisher")
    main()
