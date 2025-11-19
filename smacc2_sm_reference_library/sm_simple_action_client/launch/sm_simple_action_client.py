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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def setup_log_directory():
    """
    Creates timestamped log directory with error handling.
    Returns: (log_dir_path, timestamp) tuple
    """
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    # Primary log directory location
    log_dir = os.path.join(
        os.path.expanduser("~"), ".ros", "log", f"{timestamp}-sm_simple_action_client"
    )

    try:
        os.makedirs(log_dir, mode=0o755, exist_ok=True)
        print(f"[Launch] Log directory created: {log_dir}")
        return log_dir, timestamp
    except PermissionError as e:
        # Fallback to /tmp if ~/.ros is not writable
        fallback_dir = os.path.join("/tmp", "sm_simple_action_client_logs", timestamp)
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

    # Config file path
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("sm_simple_action_client"),
            "config",
            "simple_action_client_example_config.yaml",
        ]
    )

    # Construct logging prefix for Fibonacci action server
    if log_dir:
        fibonacci_log = os.path.join(log_dir, f"fibonacci_action_server_{timestamp}.log")
        fibonacci_prefix = f"konsole --hold -p tabtitle='Fibonacci Action Server' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {fibonacci_log}; exec bash' -- "
    else:
        fibonacci_prefix = "konsole --hold -p tabtitle='Fibonacci Action Server' -e"

    # Construct logging prefix for state machine node
    if log_dir:
        state_machine_log = os.path.join(log_dir, f"sm_simple_action_client_node_{timestamp}.log")
        state_machine_prefix = f"konsole --hold -p tabtitle='SM Simple Action Client' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {state_machine_log}; exec bash' -- "
    else:
        state_machine_prefix = "konsole --hold -p tabtitle='SM Simple Action Client' -e"

    # Construct logging prefix for auto-trigger node
    if log_dir:
        trigger_log = os.path.join(log_dir, f"auto_mode_trigger_{timestamp}.log")
        trigger_prefix = f"konsole --hold -p tabtitle='Auto Mode Trigger' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {trigger_log}; exec bash' -- "
    else:
        trigger_prefix = "konsole --hold -p tabtitle='Auto Mode Trigger' -e"

    return LaunchDescription(
        [
            # Start Fibonacci action server
            Node(
                package="action_tutorials_cpp",
                executable="fibonacci_action_server",
                name="fibonacci_action_server",
                output="screen",
                prefix=fibonacci_prefix,
            ),
            # Start state machine
            Node(
                package="sm_simple_action_client",
                executable="sm_simple_action_client_node",
                name="sm_simple_action_client",
                output="screen",
                parameters=[config_file],
                prefix=state_machine_prefix,
            ),
            # Auto-trigger autonomous mode after delay
            Node(
                package="sm_simple_action_client",
                executable="auto_mode_trigger.py",
                name="auto_mode_trigger",
                output="screen",
                prefix=trigger_prefix,
            ),
        ]
    )
