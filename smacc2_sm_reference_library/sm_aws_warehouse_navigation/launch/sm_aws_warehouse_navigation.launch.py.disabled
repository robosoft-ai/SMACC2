# Copyright (c) 2021 Samsung Research America
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_dir = get_package_share_directory("nav2_rosdevday_2021")
    robot_model_dir = get_package_share_directory("neo_simulation2")
    warehouse_dir = get_package_share_directory("aws_robomaker_small_warehouse_world")

    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    rviz_config_file = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration("use_simulator")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    urdf = LaunchConfiguration("urdf")

    # Declare the launch arguments
    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(warehouse_dir, "maps", "005", "map.yaml"),
        description="Full path to map file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "basic_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="True", description="Whether to start the simulator"
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Whether to execute gzclient for the simulation frontend",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(bringup_dir, "worlds", "industrial_sim.world"),
        description="Full path to world model file to load",
    )

    declare_urdf_cmd = DeclareLaunchArgument(
        "urdf",
        default_value=os.path.join(robot_model_dir, "robots", "mp_400", "mp_400.urdf"),
        description="Full path to world model file to load",
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=["gzserver", "-s", "libgazebo_ros_factory.so", world],
        cwd=[warehouse_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
        cmd=["gzclient"],
        cwd=[warehouse_dir],
        output="screen",
    )

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robot",
            "-file",
            urdf,
            "-x",
            "3.45",
            "-y",
            "2.15",
            "-z",
            "0.10",
            "-Y",
            "3.14",
        ],
        output="screen",
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[urdf],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
        }.items(),
    )

    sm_aws_warehouse_navigation_node = Node(
        package="sm_aws_warehouse_navigation",
        executable="sm_aws_warehouse_navigation_node",
        name="sm_aws_warehouse_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            os.path.join(
                get_package_share_directory("sm_aws_warehouse_navigation"),
                "params",
                "sm_aws_warehouse_navigation_config.yaml",
            ),
        ],
    )

    # UNCOMMENT HERE FOR KEEPOUT DEMO
    # start_lifecycle_manager_cmd = Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_costmap_filters',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[{'use_sim_time': use_sim_time},
    #                     {'autostart': True},
    #                     {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}])

    # start_map_server_cmd = Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='filter_mask_server',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[params_file])

    # start_costmap_filter_info_server_cmd = Node(
    #         package='nav2_map_server',
    #         executable='costmap_filter_info_server',
    #         name='costmap_filter_info_server',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[params_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(sm_aws_warehouse_navigation_node)
    # UNCOMMENT HERE FOR KEEPOUT DEMO
    # ld.add_action(start_lifecycle_manager_cmd)
    # ld.add_action(start_map_server_cmd)
    # ld.add_action(start_costmap_filter_info_server_cmd)

    return ld
