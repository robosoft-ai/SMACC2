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
# Author: Pablo Iñigo Blasco


from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Gazebo server
    xterm_prefix = (
        "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -sl 10000 -geometry 1000x600 -e",
    )

    gzserver = ExecuteProcess(
        cmd=["gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
        output="screen",
        prefix=xterm_prefix,
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    single_ur_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("sm_multi_ur5_sim"),
                "/launch",
                "/sm_single_ur5_sim_gazebo_sim.launch.py",
            ]
        ),
        launch_arguments={
            "prefix": "ur5_1",
            "x": "1.5",
            "y": "0.0",
            "z": "0.0",
            "description_package": "sm_multi_ur5_sim",
            "moveit_config_package": "sm_multi_ur5_sim",
            "use_state_machine": "False",
            "ros_control": "True",
            "use_moveit": "False",
        }.items(),
    )

    single_ur_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("sm_multi_ur5_sim"),
                "/launch",
                "/sm_single_ur5_sim_gazebo_sim.launch.py",
            ]
        ),
        launch_arguments={
            "prefix": "ur5_2",
            "x": "0.0",
            "y": "0.0",
            "z": "0.0",
            "description_package": "sm_multi_ur5_sim",
            "moveit_config_package": "sm_multi_ur5_sim",
            "use_state_machine": "True",
            "ros_control": "True",
            "use_moveit": "False",
        }.items(),
    )

    nodes_to_launch = [
        single_ur_launch_1,
        single_ur_launch_2,
        gzserver,
        gzclient,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    # General arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "runtime_config_package",
    #         default_value="ur_bringup",
    #         description='Package with the controller\'s configuration in "config" folder. \
    #     Usually the argument is not set, it enables use of a custom setup.',
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "controllers_file",
    #         default_value="ur_controllers.yaml",
    #         description="YAML file with the controllers configuration.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_package",
    #         default_value="ur_description",
    #         description="Description package with robot URDF/XACRO files. Usually the argument \
    #     is not set, it enables use of a custom description.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_file",
    #         default_value="ur.urdf.xacro",
    #         description="URDF/XACRO description file with the robot.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "moveit_config_package",
    #         default_value="ur_moveit_config",
    #         description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
    #     is not set, it enables use of a custom moveit config.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "moveit_config_file",
    #         default_value="ur.srdf.xacro",
    #         description="MoveIt SRDF/XACRO description file with the robot.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "prefix",
    #         default_value='""',
    #         description="Prefix of the joint names, useful for \
    #                      multi-robot setup. If changed than also joint names in the controllers' configuration \
    #                      have to be updated.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    # )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
