import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    panda_arm_1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('panda_arm_1_moveit_config'), 'launch', 'demo.launch.py')))

    panda_arm_2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('panda_arm_2_moveit_config'), 'launch', 'demo.launch.py')))

    return LaunchDescription([panda_arm_1, panda_arm_2])