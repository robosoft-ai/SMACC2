# Copyright 2026 RobosoftAI Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RUNTIME_LOG = "/tmp/sm_coretest_reactor_order_1_latest.log"


def generate_launch_description():
    tee_prefix = 'bash -c \'stdbuf -oL -eL "$@" 2>&1 | tee ' + RUNTIME_LOG + "' --"
    return LaunchDescription(
        [
            DeclareLaunchArgument("pulse_period_ms", default_value="500"),
            Node(
                package="sm_coretest_reactor_order_1",
                executable="sm_coretest_reactor_order_1_node",
                output="screen",
                prefix=tee_prefix,
                additional_env={
                    "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
                    "RCUTILS_COLORIZED_OUTPUT": "0",
                },
                parameters=[
                    {
                        "pulse_period_ms": ParameterValue(
                            LaunchConfiguration("pulse_period_ms"), value_type=int
                        ),
                    }
                ],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ]
    )
