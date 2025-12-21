import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('sm_nav2_unit_test_1')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='False')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo in headless mode if true'
    )

    # Include Nav2 TurtleBot3 simulation launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'tb3_simulation_launch.py'
            ])
        ]),
        launch_arguments={
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Keyboard server node - opens in konsole terminal for keyboard input
    keyboard_server_node = Node(
        package='cl_keyboard',
        executable='keyboard_server_node.py',
        name='keyboard_server_node',
        output='screen',
        prefix="konsole --hold -p tabtitle='Keyboard Server' -e",
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # State machine node
    sm_node = Node(
        package='sm_nav2_unit_test_1',
        executable='sm_nav2_unit_test_1_node',
        name='sm_nav2_unit_test_1',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        nav2_bringup_launch,
        keyboard_server_node,
        sm_node
    ])
