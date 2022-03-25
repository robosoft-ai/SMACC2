# Copyright (c) 2021 RobosoftAI Inc.
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

#
# Author: Denis Štogl
# Maintainer: Pablo Iñigo Blasco


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory
import math
import yaml


def construct_angle_radians(loader, node):
    """utility function to construct radian values from yaml."""
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except SyntaxError:
        raise Exception("invalid expression: %s" % value)


def construct_angle_degrees(loader, node):
    """utility function for converting degrees into radians from yaml."""
    return math.radians(construct_angle_radians(loader, node))


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception:
        raise Exception("yaml support not available; install python-yaml")

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    ur_type = "ur5e"

    # General arguments
    # runtime_config_package = LaunchConfiguration("runtime_config_package")

    runtime_config_package = LaunchConfiguration("runtime_config_package")

    # controllers_file = LaunchConfiguration("controllers_file")
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("sm_multi_ur5_sim"), "/config", "/ros_control", "/ur_controllers.yaml"]
    )
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " x:=",
            x,
            " y:=",
            y,
            " z:=",
            z,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    smacc2_sm_config = PathJoinSubstitution(
        [
            FindPackageShare("sm_multi_ur5_sim"),
            "config",
            "sm_multi_ur5_sim_config.yaml",
        ]
    )

    sm_test_moveit_ur5_sim_node = Node(
        package="sm_multi_ur5_sim",
        executable="sm_multi_ur5_sim_node",
        # prefix="xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -sl 10000 -geometry 1000x600 -e gdbserver localhost:3000",
        prefix="xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -sl 10000 -geometry 1000x600 -e",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            smacc2_sm_config,
        ],
        #namespace= "ur5_1",
        remappings=[
                    ("/joint_states", "/joint_state_broadcaster_ur5_1/joint_states"),
                    #("/move_action", "/ur5_1/move_action"),
                    ]
    )

    smacc2_sm_rviz = PathJoinSubstitution(
        [
            FindPackageShare("sm_multi_ur5_sim"),
            "rviz",
            "rviz.rviz",
        ]
    )

    # Launch rviz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", smacc2_sm_rviz],
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("sm_multi_ur5_sim"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "runtime_config_package": runtime_config_package,
            "controllers_file": PathJoinSubstitution(
                [
                    FindPackageShare("sm_multi_ur5_sim"),
                    "/config",
                    "/ros_control",
                    "/ur_controllers.yaml",
                ]
            ),
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "x": x,
            "y": y,
            "z": z,
            "launch_rviz": "false",
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("sm_multi_ur5_sim"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "use_sim_time": "true",
            "prefix": prefix,
            "x": x,
            "y": y,
            "z": z,
            "launch_rviz": "false",
        }.items(),
    )

    nodes_to_launch = [
        ur_control_launch,
        ur_moveit_launch,
        sm_test_moveit_ur5_sim_node,
        start_rviz_cmd,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
                         multi-robot setup. If changed than also joint names in the controllers' configuration \
                         have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="",
        )
    ),

    declared_arguments.append(
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="",
        )
    ),

    declared_arguments.append(
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
