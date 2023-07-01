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
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener
from tf2_ros import Buffer
import math
import tf_transformations
import std_msgs.msg


class RotationOscillation(Node):
    def __init__(self):
        super().__init__("rotation_oscillation")
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.yaw_pub = self.create_publisher(std_msgs.msg.Float64, "yaw", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.on_timer = self.create_timer(0.05, self.on_timer_callback)

        self.base_link_frame = "base_footprint"
        self.fixed_frame = "odom"

    def on_timer_callback(self):
        try:
            base_link_transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.base_link_frame, rclpy.time.Time()
            )
            q = base_link_transform.transform.rotation
            euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.get_logger().info("Current rotation: %f" % math.degrees(euler[2]))
            self.get_logger().info("Transform available!")

            t = float(self.get_clock().now().nanoseconds) / 1e9
            cmd_vel = Twist()
            cmd_vel.angular.z = 1.0 if math.sin(t) > 0.0 else -1.0
            self.cmd_vel_pub.publish(cmd_vel)

            self.yaw_pub.publish(std_msgs.msg.Float64(data=euler[2]))

            self.get_logger().info("Current time: %s" % str(t))
        except Exception as e:
            self.get_logger().error("Error getting transform: %s" % str(e))

    def oscillate_rotation(self):
        rate = self.create_rate(1)
        angle = math.radians(25)
        # timeout = rclpy.duration.Duration(seconds=1.0)  # Timeout for waiting on transforms

        fixed_frame = "odom"
        base_link_frame = "base_footprint"

        print(f'Waiting for transform from "{fixed_frame}" to "{base_link_frame}"...')
        while rclpy.ok():
            # self.get_logger().info(f'Waiting for transform from "{fixed_frame}" to "{base_link_frame}"...')
            rclpy.spin_once(self)
            # self.get_logger().info('Spin once, now: %s' % str(self.get_clock().now().toSec()))
            rate.sleep()

        self.get_logger().info("Transform available!")

        # Get initial position
        print("Getting initial position...")
        try:
            base_link_transform = self.tf_buffer.lookup_transform(
                "map", base_link_frame, rclpy.time.Time()
            )
            initial_position = base_link_transform.transform.translation
        except Exception as e:
            self.get_logger().error("Error getting initial position: %s" % str(e))
            return

        # Rotate 25 degrees to the left
        print("Rotating 25 degrees to the left...")
        self.get_logger().info("Rotating 25 degrees to the left...")
        twist_msg = Twist()
        twist_msg.angular.z = angle
        self.cmd_vel_pub.publish(twist_msg)
        rclpy.spin_once(self)

        # Get position after left rotation
        print("Getting position after left rotation...")
        try:
            print("Getting position after left rotation...")
            base_link_transform = self.tf_buffer.lookup_transform(
                "map", base_link_frame, rclpy.time.Time()
            )
            left_rotation_position = base_link_transform.transform.translation
        except Exception as e:
            self.get_logger().error("Error getting position after left rotation: %s" % str(e))
            return

        # Rotate 25 degrees to the right
        print("Rotating 25 degrees to the right...")
        self.get_logger().info("Rotating 25 degrees to the right...")
        twist_msg.angular.z = -angle
        self.cmd_vel_pub.publish(twist_msg)
        rclpy.spin_once(self)

        # Get position after right rotation
        try:
            base_link_transform = self.tf_buffer.lookup_transform(
                "map", base_link_frame, rclpy.time.Time()
            )
            right_rotation_position = base_link_transform.transform.translation
        except Exception as e:
            self.get_logger().error("Error getting position after right rotation: %s" % str(e))
            return

        # Calculate phase difference
        print("Calculating phase difference...")
        left_rotation_angle = math.atan2(
            left_rotation_position.y - initial_position.y,
            left_rotation_position.x - initial_position.x,
        )
        right_rotation_angle = math.atan2(
            right_rotation_position.y - initial_position.y,
            right_rotation_position.x - initial_position.x,
        )
        phase_difference = math.degrees(right_rotation_angle - left_rotation_angle)

        self.get_logger().info("Phase difference: %f degrees" % phase_difference)

        # Stop the robot
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rotation_oscillation = RotationOscillation()

    rclpy.spin(rotation_oscillation)

    try:
        rotation_oscillation.oscillate_rotation()
    except Exception as e:
        rotation_oscillation.get_logger().error("Error during rotation oscillation: %s" % str(e))

    rotation_oscillation.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
