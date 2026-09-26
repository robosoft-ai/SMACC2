# Copyright 2026 RobosoftAI Inc.
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

# Terminal 4 of sm_cl_px4_mr_test_5. Gazebo + PX4 (start_cave_sitl.sh), QGC and
# the micro-ROS agent are started separately. This launch brings up:
#   1. the gz -> ROS clock bridge          (/clock; everything here runs on sim time)
#   2. the gz -> ROS lidar bridge           (/lidar_3d/points -> /lidar/points, PointCloud2)
#   3. RViz with the cave view              (fixed frame map, cloud live + accumulated)
#   4. the state machine, after a delay so /clock is flowing before its node starts
#
# Every run's complete console output (stdout + stderr, unbuffered, no color
# codes) is tee'd to this fixed path so it can be read after the fact.

import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RUNTIME_LOG = "/tmp/sm_cl_px4_mr_test_5_latest.log"
# every run also keeps a dated copy of the same console log, and the map recorder
# writes the accumulated lidar voxel map (map frame, ENU) for offline analysis
RUN_STAMP = time.strftime("%Y%m%d_%H%M%S")
RUNTIME_LOG_DATED = f"/tmp/sm_cl_px4_mr_test_5_{RUN_STAMP}.log"
MAP_FILE_LATEST = "/tmp/sm_cl_px4_mr_test_5_map.npy"
MAP_FILE_DATED = f"/tmp/sm_cl_px4_mr_test_5_map_{RUN_STAMP}.npy"


def generate_launch_description():
    lidar_gz_topic = LaunchConfiguration("lidar_gz_topic")
    lidar_ros_topic = LaunchConfiguration("lidar_ros_topic")
    rviz = LaunchConfiguration("rviz")
    sm_start_delay = LaunchConfiguration("sm_start_delay")

    tee_prefix = (
        'bash -c \'stdbuf -oL -eL "$@" 2>&1 | tee ' + RUNTIME_LOG + " " + RUNTIME_LOG_DATED + "' --"
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("sm_cl_px4_mr_test_5"), "config", "sm_cl_px4_mr_test_5.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_gz_topic",
                default_value="/lidar_3d/points",
                description="gz topic of the lidar_3d sensor's PointCloudPacked",
            ),
            DeclareLaunchArgument(
                "lidar_ros_topic",
                default_value="/lidar/points",
                description="ROS topic the cloud is bridged to (OrLidar subscribes here)",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "record_map", default_value="true", description="accumulate the lidar voxel map to /tmp"
            ),
            DeclareLaunchArgument(
                "sm_start_delay",
                default_value="5.0",
                description="seconds to wait for /clock before starting the state machine",
            ),
            # separate processes on purpose: killing the lidar bridge (a negative
            # test) must not stop /clock, or the sim-time watchdogs freeze
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="lidar_bridge",
                output="screen",
                arguments=[
                    [lidar_gz_topic, "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
                    "--ros-args",
                    "-r",
                    [lidar_gz_topic, ":=", lidar_ros_topic],
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(rviz),
            ),
            Node(
                package="sm_cl_px4_mr_test_5",
                executable="cloud_map_recorder.py",
                name="cloud_map_recorder",
                output="screen",
                parameters=[{"use_sim_time": True, "topic": lidar_ros_topic, "output": MAP_FILE_DATED}],
                condition=IfCondition(LaunchConfiguration("record_map")),
            ),
            Node(
                package="sm_cl_px4_mr_test_5",
                executable="cloud_map_recorder.py",
                name="cloud_map_recorder_latest",
                output="log",
                parameters=[{"use_sim_time": True, "topic": lidar_ros_topic, "output": MAP_FILE_LATEST}],
                condition=IfCondition(LaunchConfiguration("record_map")),
            ),
            TimerAction(
                period=sm_start_delay,
                actions=[
                    Node(
                        package="sm_cl_px4_mr_test_5",
                        executable="sm_cl_px4_mr_test_5_node",
                        output="screen",
                        prefix=tee_prefix,
                        additional_env={
                            "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
                            "RCUTILS_COLORIZED_OUTPUT": "0",
                        },
                        parameters=[{"use_sim_time": True}],
                        arguments=["--ros-args", "--log-level", "INFO"],
                    ),
                ],
            ),
        ]
    )
