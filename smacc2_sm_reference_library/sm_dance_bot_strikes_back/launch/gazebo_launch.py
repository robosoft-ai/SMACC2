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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="False", description="Whether to execute gzclient)"
    )

    use_simulator = LaunchConfiguration("use_simulator")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    sm_dance_bot_strikes_back_dir = get_package_share_directory("sm_dance_bot_strikes_back")
    launch_dir = os.path.join(sm_dance_bot_strikes_back_dir, "launch")

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="True", description="Whether to start the simulator"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        # default_value=os.path.join(sm_dance_bot_strikes_back_dir, 'worlds', 'waffle.model'),
        # default_value=os.path.join(sm_dance_bot_strikes_back_dir, 'worlds', 'sm_dance_bot_strikes_back_world.model'),
        default_value=os.path.join(
            sm_dance_bot_strikes_back_dir, "worlds", "ridgeback_race.world"
        ),
        description="Full path to world model file to load",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' " \
    # "-hold -geometry 1000x600 -sl 10000 -e"

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=["gzserver", "-s", "libgazebo_ros_init.so", world, "--verbose"],
        # env={"GAZEBO_MODEL_PATH": os.getcwd(),"HOME": os.environ["HOME"]},
        cwd=[launch_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen",
    )

    # Add any conditioned actions
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
