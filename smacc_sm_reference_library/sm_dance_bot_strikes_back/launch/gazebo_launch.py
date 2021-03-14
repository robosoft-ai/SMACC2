
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_simulator_cmd = DeclareLaunchArgument(
    'use_simulator',
    default_value='False',
    description='Whether to execute gzclient)')

    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    sm_dance_bot_dir = get_package_share_directory('sm_dance_bot_strikes_back')
    launch_dir = os.path.join(sm_dance_bot_dir, 'launch')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
        
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        #default_value=os.path.join(sm_dance_bot_dir, 'worlds', 'waffle.model'),
        #default_value=os.path.join(sm_dance_bot_dir, 'worlds', 'sm_dance_bot_world.model'),
        default_value=os.path.join(sm_dance_bot_dir, 'worlds', 'ridgeback_race.world'),
        description='Full path to world model file to load')
    

    # Create the launch description and populate
    ld = LaunchDescription()

        # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    # Add any conditioned actions
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld