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
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    panda_arm_1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_arm_1_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        )
    )

    panda_arm_2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_arm_2_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        )
    )

    keyboard_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sm_multi_panda_sim"),
                "launch",
                "keyboard_server.launch.py",
            )
        )
    )

    return LaunchDescription([panda_arm_1, panda_arm_2, keyboard_server])
