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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup_log_directory():
    """
    Creates timestamped log directory with error handling.
    Returns: (log_dir_path, timestamp) tuple
    """
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    # Primary log directory location
    log_dir = os.path.join(os.path.expanduser("~"), ".ros", "log", f"{timestamp}-sm_branching")

    try:
        os.makedirs(log_dir, mode=0o755, exist_ok=True)
        print(f"[Launch] Log directory created: {log_dir}")
        return log_dir, timestamp
    except PermissionError as e:
        # Fallback to /tmp if ~/.ros is not writable
        fallback_dir = os.path.join("/tmp", "sm_branching_logs", timestamp)
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
    # Setup logging directory
    log_dir, timestamp = setup_log_directory()

    # Get package share directory for config file
    package_share_dir = get_package_share_directory("sm_branching")
    config_file = os.path.join(package_share_dir, "config", "sm_branching_config.yaml")

    # Construct logging prefix for state machine node
    if log_dir:
        state_machine_log = os.path.join(log_dir, f"state_machine_{timestamp}.log")
        state_machine_prefix = f"konsole --hold -p tabtitle='SM Branching' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {state_machine_log}; exec bash' -- "
    else:
        state_machine_prefix = "konsole --hold -p tabtitle='SM Branching' -e"

    return LaunchDescription(
        [
            Node(
                package="sm_branching",
                executable="sm_branching_node",
                name="sm_branching",
                output="screen",
                prefix=state_machine_prefix,
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ],
    )
