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

# Every run's complete console output (stdout + stderr, unbuffered, no color
# codes) is tee'd to this fixed path so it can be read after the fact without
# hunting through ~/.ros/log. The ROS node log there is still written as well.
RUNTIME_LOG = "/tmp/sm_cl_px4_mr_test_4_latest.log"


def generate_launch_description():
    # test_leg: "" (default) flies the full mission. Any superstate or nav
    # state name (SsSquareSpiral1, SsSquareSpiral2, SsLawnmower1, SsLawnmower2,
    # SsGridPattern1, SsVSSearch1/2/3, SsVSChain1/2/3, StSineWaveVertical, StSineWaveHorizontal,
    # StGoToLandingZone) runs only that leg, centred on the takeoff point, and
    # then lands at home.
    test_leg_arg = DeclareLaunchArgument(
        "test_leg",
        default_value="",
        description="Run a single railway leg in place (empty = full mission)",
    )

    tee_prefix = "bash -c 'stdbuf -oL -eL \"$@\" 2>&1 | tee " + RUNTIME_LOG + "' --"

    return LaunchDescription(
        [
            test_leg_arg,
            Node(
                package="sm_cl_px4_mr_test_4",
                executable="sm_cl_px4_mr_test_4_node",
                output="screen",
                prefix=tee_prefix,
                additional_env={
                    "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
                    "RCUTILS_COLORIZED_OUTPUT": "0",
                },
                parameters=[{"test_leg": LaunchConfiguration("test_leg")}],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ]
    )
