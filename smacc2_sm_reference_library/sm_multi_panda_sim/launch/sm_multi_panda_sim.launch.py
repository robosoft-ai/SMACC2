import os
from sys import prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("dual_panda", package_name="sm_multi_panda_sim")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    xtermprefix = (
        "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' "
        "-hold -geometry 1000x600 -sl 10000 -e"
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        prefix=xtermprefix,
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("sm_multi_panda_sim"),
        "launch/moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    sm_multi_panda_sim = Node(
        package="sm_multi_panda_sim",
        executable="sm_multi_panda_sim_node",
        prefix=xtermprefix,
        parameters=[moveit_config.to_dict()],
    )

    # MoveItCpp demo executable
    # moveit_cpp_node = Node(
    #     name="moveit_cpp_tutorial",
    #     package="sm_multi_panda_sim",
    #     executable="moveit_cpp_tutorial_dual",
    #     output="screen",
    #     prefix = xtermprefix,
    #     parameters=[moveit_config.to_dict()],
    # )

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
        get_package_share_directory("sm_multi_panda_sim"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        prefix=xtermprefix,
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_arm_controller",
        "right_arm_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {controller}"],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            # moveit_cpp_node
            sm_multi_panda_sim,
        ]
        + load_controllers
    )
