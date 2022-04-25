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

    # parameters_file_dir = os.path.join( get_package_share_directory("robot_localization"), "params")
    # parameters_file_path = os.path.join(parameters_file_dir, "dual_ekf_navsat_example.yaml")

    parameters_file_dir = os.path.join(
        get_package_share_directory("sm_husky_barrel_search_1"), "params", "nav2z_client"
    )
    parameters_file_path = os.path.join(parameters_file_dir, "dual_ekf_navsat.yaml")

    os.environ["FILE_PATH"] = str(parameters_file_dir)

    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument("use_sim_time", default_value="true"),
            launch.actions.DeclareLaunchArgument("output_final_position", default_value="false"),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    parameters_file_path,
                ],
                remappings=[
                    ("odometry/filtered", "odometry/local"),
                    ("odometry/wheel", "odom"),
                    ("imu/data", "imu"),
                ],
                prefix=xtermprefix,
                arguments=["--log-level", "INFO"],
            ),
            # launch_ros.actions.Node(
            #     package="robot_localization",
            #     executable="ekf_node",
            #     name="ekf_filter_node_map",
            #     output="screen",
            #     parameters=[
            #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
            #         parameters_file_path,
            #     ],
            #     remappings=
            #     [
            #         ("odometry/filtered", "odometry/global"),
            #         ("odometry/wheel", "/odom"),
            #         ("imu/data", "imu")
            #     ],
            #     prefix=xtermprefix,
            #     arguments= ["--log-level", "INFO"]
            # ),
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
                arguments=[
                    "--ros-args",
                    "-p",
                    "broadcast_utm_transform:=true",
                    "--log-level",
                    "INFO",
                ],
            ),
        ]
    )
