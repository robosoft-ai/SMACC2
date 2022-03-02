# Copyright 2018 Open Source Robotics Foundation, Inc.
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
import launch_ros.actions
import os
from launch.substitutions import LaunchConfiguration
import launch.actions

# from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_localization_dir = get_package_share_directory("robot_localization")
    parameters_file_dir = os.path.join(robot_localization_dir, "params")
    # parameters_file_path = os.path.join("sm_husky_barrel_search_1", 'dual_ekf_navsat.yaml')

    parameters_file_path = os.path.join(parameters_file_dir, "dual_ekf_navsat_example.yaml")
    os.environ["FILE_PATH"] = str(parameters_file_dir)

    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"),
            launch.actions.DeclareLaunchArgument("output_final_position", default_value="false"),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            # launch_ros.actions.Node(
            #         package='robot_localization',
            #         executable='ekf_node',
            #         name='ekf_filter_node_odom',
            #         output='screen',
            #         parameters=[parameters_file_path],
            #         remappings=[('odometry/filtered', 'odometry/local')]
            #        ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    parameters_file_path,
                ],
                remappings=[("odometry/filtered", "odometry/global"), ("odometry/wheel", "/odom")],
                prefix=xtermprefix,
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    parameters_file_path,
                ],
                remappings=[
                    ("imu/data", "imu"),
                    ("gps/fix", "gps/data"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                prefix=xtermprefix,
            ),
        ]
    )
