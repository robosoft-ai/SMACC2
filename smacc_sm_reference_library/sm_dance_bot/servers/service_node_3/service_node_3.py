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

# import roslib
# import rospy
import rclpy
from rclpy.node import Node

import std_srvs

if __name__ == "__main__":

    class Service3(Node):
        def __init__(self):
            super().__init__("Service3")
            self.s = self.create_service(
                std_srvs.srv.SetBool, "service_node3", self.set_bool_service
            )

        def set_bool_service(self, req, res):
            self.get_logger().info("RECEIVING SET BOOL SERVICE REQUEST: value=" + str(req.data))

            res.message = "OK, value set"
            res.success = True
            return res

    rclpy.init(args=None)
    s = Service3()

    # s = rospy.Service('service_node3', std_srvs.srv.SetBool, set_bool_service)
    # rospy.spin()
    rclpy.spin(s)
    rclpy.shutdown()
