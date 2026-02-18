from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sm_cl_px4_mr_test_1",
                executable="sm_cl_px4_mr_test_1_node",
                output="screen",
                arguments=["--ros-args", "--log-level", "INFO"],
            )
        ]
    )
