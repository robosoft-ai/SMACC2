# Copyright (c) 2023 Open Navigation LLC
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')

    world = os.path.join(coverage_demo_dir, 'blank.world')
    param_file_path = os.path.join(coverage_demo_dir, 'demo_params.yaml')
    sdf = os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model')

    # start the simulation
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[coverage_demo_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
         cmd=['gzclient'],
         cwd=[coverage_demo_dir], output='screen')

    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}])

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'tb3',
            '-file', sdf,
            '-x', '5.0', '-y', '5.0', '-z', '0.10',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'])

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': ''}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    # world->odom transform, no localization. For visualization & controller transform
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    # # start the demo task
    # demo_cmd = Node(
    #     package='opennav_coverage_demo',
    #     executable='demo_coverage',
    #     emulate_tty=True,
    #     output='screen')
    

    xtermprefix = "xterm -hold -e "

    sm_dance_bot_coverage_node = Node(
        package="sm_dance_bot_coverage",
        executable="sm_dance_bot_coverage_node",
        name="SmDanceBotCoverage",
        output="screen",
        prefix=xtermprefix,
        parameters=[
            os.path.join(
                get_package_share_directory("sm_dance_bot_coverage"),
                "params/sm_dance_bot_config.yaml",
            )
        ],
        remappings=[
            # ("/odom", "/odometry/filtered"),
            # ("/sm_dance_bot_2/odom_tracker/odom_tracker_path", "/odom_tracker_path"),
            # ("/sm_dance_bot_2/odom_tracker/odom_tracker_stacked_path", "/odom_tracker_path_stacked")
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(sm_dance_bot_coverage_node)
    #ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(fake_localization_cmd)
    # ld.add_action(demo_cmd)
    return ld
