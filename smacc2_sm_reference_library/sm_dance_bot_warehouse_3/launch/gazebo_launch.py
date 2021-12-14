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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions.node import Node


def generate_launch_description():
    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="False", description="Whether to execute gzclient)"
    )

    use_simulator = LaunchConfiguration("use_simulator")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    show_gz_lidar = LaunchConfiguration("show_gz_lidar")

    sm_dance_bot_warehouse_3_dir = get_package_share_directory("sm_dance_bot_warehouse_3")
    launch_dir = os.path.join(sm_dance_bot_warehouse_3_dir, "launch")

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="True", description="Whether to start the simulator"
    )

    declare_headless_simulator_argument = DeclareLaunchArgument(
        "headless", default_value="True", description="Whether to execute gzclient)"
    )

    declare_show_gz_lidar = DeclareLaunchArgument(
        "show_gz_lidar",
        default_value="True",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sm_dance_bot_warehouse_3_dir, "worlds", "industrial_sim.world"),
        description="Full path to world model file to load",
        condition=IfCondition(show_gz_lidar),
    )

    # nolidar world
    declare_world_cmd_2 = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sm_dance_bot_warehouse_3_dir, "worlds", "industrial_sim.world"),
        description="Full path to world model file to load",
        condition=UnlessCondition(show_gz_lidar),
    )

    declare_urdf = DeclareLaunchArgument(
        "urdf",
        default_value=os.path.join(
            sm_dance_bot_warehouse_3_dir, "models", "turtlebot3_waffle", "model.sdf"
        ),
        description="",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    xtermprefix = (
        "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' "
        "-hold -geometry 1000x600 -sl 10000 -e"
    )

    gzenv = dict(os.environ)
    model_database_uri = os.environ["GAZEBO_MODEL_PATH"]
    gzenv["GAZEBO_MODEL_DATABASE_URI"] = model_database_uri

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
            "--verbose",
        ],
        env=gzenv,
        cwd=[launch_dir],
        output="screen",
        prefix=xtermprefix,
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
        cmd=["gzclient"],
        cwd=[launch_dir],
        env=gzenv,
        output="screen",
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="gazebo_ros",
        arguments=[
            "-entity",
            "turtlebot3_waffle",
            "-file",
            LaunchConfiguration("urdf"),
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
            "-Y",
            "0",
        ],
    )

    # Add any conditioned actions
    ld.add_action(declare_headless_simulator_argument)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_cmd_2)
    ld.add_action(declare_urdf)

    ld.add_action(declare_show_gz_lidar)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_node)

    return ld
