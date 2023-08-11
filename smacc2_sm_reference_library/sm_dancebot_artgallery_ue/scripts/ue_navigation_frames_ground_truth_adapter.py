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


class StaticTransformPublisher:
    def __init__(self, parent_frame, child_frame):
        self.node = rclpy.create_node("static_transform_publisher_1")
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(node)
        self.clock_msg = None
        print("Initializing static transform publisher")

        self.uemsgs_sub = node.create_subscription(
            ue_msgs.msg.EntityState,
            "/ue_ros/model_state",
            self.uemsgs_callback,

            # reliability reliable
            rclpy.qos.qos_profile_services_default
        )

        self.clock_sub = node.create_subscription(
            rosgraph_msgs.msg.Clock,
            "/clock",
            self.clock_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

    def clock_callback(self, msg):
        clock_msg = msg

    def clock_callback(self, msg):
        self.clock_msg = msg


def main(args=None):
    print("Initializing static transform publisher")
    rclpy.init()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    print("Starting static transform publisher")

    main()
