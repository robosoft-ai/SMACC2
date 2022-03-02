# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sm_dance_bot_warehouse_dir = get_package_share_directory("sm_dance_bot_warehouse")
    sm_dance_bot_warehouse_launch_dir = os.path.join(sm_dance_bot_warehouse_dir, "launch")

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")
    autostart = LaunchConfiguration("autostart")
    show_gz_lidar = LaunchConfiguration("show_gz_lidar")
    headless = LaunchConfiguration("headless")

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    urdf = os.path.join(sm_dance_bot_warehouse_dir, "urdf", "turtlebot3_waffle.urdf")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_headless_simulator_argument = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_show_gz_lidar = DeclareLaunchArgument(
        "show_gz_lidar",
        default_value="true",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            sm_dance_bot_warehouse_dir, "params", "nav2z_client", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_nav_to_pose_bt_xml",
        default_value=os.path.join(
            sm_dance_bot_warehouse_dir, "params", "nav2z_client", "navigation_tree.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(sm_dance_bot_warehouse_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(sm_dance_bot_warehouse_dir, "maps", "turtlebot3_world.yaml"),
        description="Full path to map file to load",
    )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=remappings,
        arguments=[urdf],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sm_dance_bot_warehouse_launch_dir, "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sm_dance_bot_warehouse_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "autostart": autostart,
            "params_file": params_file,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sm_dance_bot_warehouse_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "autostart": autostart,
            "params_file": params_file,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
        }.items(),
    )

    # gazebo_husky = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(sm_dance_bot_warehouse_launch_dir, "husky_gazebo.launch.py")),
    #     launch_arguments={'world_path': os.path.join(sm_dance_bot_warehouse_dir, "worlds", "ridgeback_race_empty.world")}.items()
    # )

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sm_dance_bot_warehouse_launch_dir, "gazebo_launch.py")
        ),
        launch_arguments={"show_gz_lidar": show_gz_lidar, "headless": headless}.items(),
    )

    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"

    sm_dance_bot_warehouse_node = Node(
        package="sm_dance_bot_warehouse",
        executable="sm_dance_bot_warehouse_node",
        name="SmDanceBotWarehouse",
        output="screen",
        prefix=xtermprefix,
        parameters=[
            os.path.join(
                get_package_share_directory("sm_dance_bot_warehouse"),
                "params/sm_dance_bot_warehouse_config.yaml",
            )
        ],
        remappings=[
            # ("/odom", "/odometry/filtered"),
            # ("/sm_dance_bot_warehouse_2/odom_tracker/odom_tracker_path", "/odom_tracker_path"),
            # ("/sm_dance_bot_warehouse_2/odom_tracker/odom_tracker_stacked_path", "/odom_tracker_path_stacked")
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    led_action_server_node = Node(
        package="sm_dance_bot_warehouse",
        executable="led_action_server_node",
        output="screen",
        prefix=xtermprefix,
    )

    temperature_action_server = Node(
        package="sm_dance_bot_warehouse",
        executable="temperature_sensor_node",
        output="screen",
        prefix=xtermprefix,
    )

    service3_node = Node(
        package="sm_dance_bot_warehouse",
        executable="service_node_3.py",
        output="screen",
        prefix=xtermprefix,
        parameters=[
            {"autostart": True, "node_names": ["ss", "dfa"]},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_show_gz_lidar)
    ld.add_action(declare_headless_simulator_argument)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(gazebo_simulator)
    # ld.add_action(gazebo_husky)

    ld.add_action(sm_dance_bot_warehouse_node)
    ld.add_action(service3_node)
    ld.add_action(temperature_action_server)
    ld.add_action(led_action_server_node)

    # # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
