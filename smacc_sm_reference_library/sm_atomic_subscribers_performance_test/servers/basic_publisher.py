#!/usr/bin/env python3

# Copyright 2021 RobosoftAI Inc.
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
from std_msgs.msg import Int16
import random


class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__("temperature_sensor")
        self.temperature_publisher_ = self.create_publisher(Int16, "temperature", 10)
        self.temperature_timer_ = self.create_timer(0.05, self.publish_temperature)

    def publish_temperature(self):
        temperature = random.randint(20, 30)
        msg = Int16()
        msg.data = temperature
        self.temperature_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
