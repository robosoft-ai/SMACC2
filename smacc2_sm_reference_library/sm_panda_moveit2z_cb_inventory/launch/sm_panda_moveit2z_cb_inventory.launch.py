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
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def setup_log_directory():
    """
    Creates timestamped log directory with error handling.
    Returns: (log_dir_path, timestamp) tuple
    """
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    # Primary log directory location
    log_dir = os.path.join(os.path.expanduser("~"), ".ros", "log", timestamp)

    try:
        os.makedirs(log_dir, mode=0o755, exist_ok=True)
        print(f"[Launch] Log directory created: {log_dir}")
        return log_dir, timestamp
    except PermissionError as e:
        # Fallback to /tmp if ~/.ros is not writable
        fallback_dir = os.path.join("/tmp", "sm_panda_moveit2z_cb_inventory_logs", timestamp)
        print(f"[Launch] WARNING: Cannot create log directory at {log_dir}")
        print(f"[Launch] Permission denied: {e}")
        print(f"[Launch] Using fallback directory: {fallback_dir}")
        try:
            os.makedirs(fallback_dir, mode=0o755, exist_ok=True)
            return fallback_dir, timestamp
        except Exception as fallback_error:
            print(f"[Launch] ERROR: Cannot create fallback directory: {fallback_error}")
            print(f"[Launch] Logs will only be displayed in konsole terminals")
            return None, timestamp
    except OSError as e:
        print(f"[Launch] ERROR: Failed to create log directory: {e}")
        print(f"[Launch] Logs will only be displayed in konsole terminals")
        return None, timestamp


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="panda_moveit_config_demo.rviz",
            description="RViz configuration file",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):

    # Setup logging directory
    log_dir, timestamp = setup_log_directory()

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    # Construct logging prefix
    if log_dir:
        move_group_log = os.path.join(log_dir, f"move_group_{timestamp}.log")
        move_group_prefix = f"konsole --hold -p tabtitle='Move Group' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {move_group_log}; exec bash' -- "
    else:
        move_group_prefix = "konsole --hold -p tabtitle='Move Group' -e"

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        prefix=move_group_prefix,
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("sm_panda_moveit2z_cb_inventory"), "launch", rviz_base]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # Construct logging prefix for state machine node
    if log_dir:
        state_machine_log = os.path.join(log_dir, f"state_machine_{timestamp}.log")
        state_machine_prefix = f"konsole --hold -p tabtitle='State Machine' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {state_machine_log}; exec bash' -- "
    else:
        state_machine_prefix = "konsole --hold -p tabtitle='State Machine' -e"

    smacc_state_machine_spawner = Node(
        package="sm_panda_moveit2z_cb_inventory",
        executable="sm_panda_moveit2z_cb_inventory_node",
        prefix=state_machine_prefix,
        output="screen",
    )

    # Construct logging prefix for keyboard client node
    if log_dir:
        keyboard_log = os.path.join(log_dir, f"keyboard_client_{timestamp}.log")
        keyboard_prefix = f"konsole --hold -p tabtitle='Keyboard Client' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {keyboard_log}; exec bash' -- "
    else:
        keyboard_prefix = "konsole --hold -p tabtitle='Keyboard Client' -e"

    keyboard_client_node = Node(
        package="cl_keyboard",
        executable="keyboard_server_node.py",
        name="keyboard_client",
        output="screen",
        prefix=keyboard_prefix,
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )
    nodes_to_start = [
        smacc_state_machine_spawner,
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
        keyboard_client_node,
    ]

    return nodes_to_start
