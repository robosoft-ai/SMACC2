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


class StaticTransformPublisher(Node):
    def __init__(self, name, parent_frame, child_frame, xyz=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]):
        super().__init__(name)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.01, self.publish_transform)
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

    def publish_transform(self):
        self.transform.header.stamp = rclpy.time.Time(0)
        self.transform_broadcaster.sendTransform(self.transform)


def main(args=None):
    rclpy.init(args=args)

    # Create the first instance of StaticTransformPublisher
    node1 = StaticTransformPublisher(
        "static_transform_publisher_1",
        "base_footprint",
        "base_link",
    )

    # Create the second instance of StaticTransformPublisher
    node2 = StaticTransformPublisher(
        "static_transform_publisher_2",
        "base_link",
        "base_scan",
        xyz=[-0.064, 0.0, 0.122],
    )

    try:
        # Spin both nodes simultaneously
        while rclpy.ok():
            rclpy.spin_once(node1)
            rclpy.spin_once(node2)
    except KeyboardInterrupt:
        pass

    node1.destroy_node()
    node2.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
