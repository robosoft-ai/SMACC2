# Copyright (c) 2020 Samsung Research Russia
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
from launch.substitutions import LaunchConfiguration

# from launch_ros.actions import Node
# from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Input parameters declaration
    # namespace = LaunchConfiguration("namespace")
    # params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # autostart = LaunchConfiguration("autostart")
    # sm_husky_barrel_search_1 = get_package_share_directory("sm_husky_barrel_search_1")

    # Variables
    # lifecycle_nodes = ["slam_toolbox"]

    # Getting directories and launch-files
    # bringup_dir = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    # slam_launch_file = os.path.join(sm_husky_barrel_search_1, 'launch', 'online_sync_launch.py')
    slam_launch_file = os.path.join(slam_toolbox_dir, "launch", "online_sync_launch.py")

    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {"use_sim_time": use_sim_time}

    # configured_params = RewrittenYaml(
    # source_file=params_file,
    # root_key=namespace,
    # param_rewrites=param_substitutions,
    # convert_types=True,
    # )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation (Gazebo) clock if true"
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the nav2 stack"
    )

    # Nodes launching commands

    # xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' " \
    # "-hold -geometry 1000x600 -sl 10000 -e"

    # start_lifecycle_manager_cmd = Node(
    # package="nav2_lifecycle_manager",
    # executable="lifecycle_manager",
    # name="lifecycle_manager_slam",
    # output="screen",
    # parameters=[
    # {"use_sim_time": use_sim_time},
    # {"autostart": autostart},
    # {"node_names": lifecycle_nodes},
    # ],
    # prefix=xtermprefix,
    # )

    # If the provided param file doesn't have slam_toolbox params, we must remove the 'params_file'
    # LaunchConfiguration, or it will be passed automatically to slam_toolbox and will not load
    # the default file

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Pop (or load) previous LaunchConfiguration, resetting the state of params_file
    # pop_launch_config = PopLaunchConfigurations(
    #         condition=UnlessCondition(has_slam_toolbox_params))

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    # Running Map Saver Server
    # ld.add_action(start_map_saver_server_cmd)
    # ld.add_action(start_lifecycle_manager_cmd)

    # Running SLAM Toolbox
    # ld.add_action(push_launch_config)
    # ld.add_action(remove_params_file)
    ld.add_action(start_slam_toolbox_cmd)
    # ld.add_action(pop_launch_config)

    return ld
