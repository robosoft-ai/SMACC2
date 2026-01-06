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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory("sm_nav2_gazebo_test_1")

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    headless = LaunchConfiguration("headless", default="False")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
    )

    declare_headless = DeclareLaunchArgument(
        "headless", default_value="False", description="Run Gazebo in headless mode if true"
    )

    # Include Nav2 TurtleBot3 simulation launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "tb3_simulation_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "headless": headless,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Keyboard server node - opens in konsole terminal for keyboard input
    keyboard_server_node = Node(
        package="cl_keyboard",
        executable="keyboard_server_node.py",
        name="keyboard_server_node",
        output="screen",
        prefix="konsole --hold -p tabtitle='Keyboard Server' -e",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # State machine node
    sm_node = Node(
        package="sm_nav2_gazebo_test_1",
        executable="sm_nav2_gazebo_test_1_node",
        name="sm_nav2_gazebo_test_1",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_headless,
            nav2_bringup_launch,
            keyboard_server_node,
            sm_node,
        ]
    )
