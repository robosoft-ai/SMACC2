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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("panda_arm_1")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict() | {"ros_control_namespace": "/panda_arm_1"}],
        arguments=["--ros-args", "--log-level", "move_group:=DEBUG"],
        prefix="xterm -hold -e",
    )

    # RViz
    # rviz_config = os.path.join(
    #     get_package_share_directory("panda_arm_1_moveit_config"),
    #     "launch/moveit.rviz",
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.planning_pipelines,
    #         moveit_config.robot_description_kinematics,
    #         {"ros_control_namespace": "/panda_arm_1"},
    #     ],
    # )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        prefix="xterm -hold -e",
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("panda_arm_1_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
        arguments=["--ros-args", "--log-level", "debug"],
        prefix="xterm -hold -e",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_arm_controller",
        # "right_arm_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=[
                    f"ros2 run controller_manager spawner {controller}"
                    + " -n panda_arm_1 -c panda_arm_1/controller_manager"
                ],
                shell=True,
                output="screen",
            )
        ]

    namespace = GroupAction(
        actions=[
            # rviz_node,
            PushRosNamespace("panda_arm_1"),
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )

    return LaunchDescription([namespace])
