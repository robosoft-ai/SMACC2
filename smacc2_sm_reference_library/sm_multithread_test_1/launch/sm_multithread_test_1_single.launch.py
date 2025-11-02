# Copyright 2025 RobosoftAI Inc.
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
Launch file for single-threaded executor comparison

This launches the same state machine with ExecutionModel::SINGLE_THREAD_SPINNER,
using traditional single-threaded callback processing.

Expected behavior:
- Same thread ID for all callbacks
- No overlapping START/END timestamps
- Serial execution of timer callbacks

To compare with multi-threaded mode:
  ros2 launch sm_multithread_test_1 sm_multithread_test_1.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sm_multithread_test_1",
                executable="sm_multithread_test_1_node_single",  # Single-threaded variant
                output="screen",
                emulate_tty=True,  # Better log formatting
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ]
    )
