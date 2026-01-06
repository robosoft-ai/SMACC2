#!/usr/bin/env python3
# Copyright 2025 Robosoft Inc.
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

"""
Auto-triggers the state machine transition for demo purposes.
Publishes autonomous mode command after a delay.
"""
import time
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32


class AutoModeTrigger(Node):
    def __init__(self):
        super().__init__("auto_mode_trigger")
        self.publisher = self.create_publisher(Int32, "mode_command", 10)

        # Wait for state machine to initialize
        self.get_logger().info(
            "╔════════════════════════════════════════════════════════════════╗"
        )
        self.get_logger().info("║  AUTO MODE TRIGGER - Waiting 3 seconds for SM initialization  ║")
        self.get_logger().info(
            "╚════════════════════════════════════════════════════════════════╝"
        )
        time.sleep(3.0)

        # Publish autonomous mode command
        msg = Int32()
        msg.data = 1  # Autonomous mode
        self.publisher.publish(msg)
        self.get_logger().info(" ")
        self.get_logger().info(
            "╔════════════════════════════════════════════════════════════════╗"
        )
        self.get_logger().info(
            "║  ✓ Published AUTONOMOUS mode command (data=1)                  ║"
        )
        self.get_logger().info(
            "║  State machine should transition: StState1 → StState2          ║"
        )
        self.get_logger().info(
            "╚════════════════════════════════════════════════════════════════╝"
        )
        self.get_logger().info(" ")

        # Schedule shutdown after publishing
        time.sleep(1.0)
        self.get_logger().info("Auto-trigger complete. Node shutting down.")
        # Create a timer to shutdown after allowing message to be sent
        self.create_timer(0.5, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = AutoModeTrigger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
