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


def setup_log_directory():
    """
    Creates timestamped log directory with error handling.
    Returns: (log_dir_path, timestamp) tuple
    """
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")

    # Primary log directory location
    log_dir = os.path.join(
        os.path.expanduser("~"), ".ros", "log", f"{timestamp}-sm_atomic_lifecycle"
    )

    try:
        os.makedirs(log_dir, mode=0o755, exist_ok=True)
        print(f"[Launch] Log directory created: {log_dir}")
        return log_dir, timestamp
    except PermissionError as e:
        # Fallback to /tmp if ~/.ros is not writable
        fallback_dir = os.path.join("/tmp", "sm_atomic_lifecycle_logs", timestamp)
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

    # Construct logging prefix for state machine node
    if log_dir:
        state_machine_log = os.path.join(log_dir, f"sm_atomic_lifecycle_node_{timestamp}.log")
        state_machine_prefix = f"konsole --hold -p tabtitle='SM Atomic Lifecycle' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {state_machine_log}; exec bash' -- "
    else:
        state_machine_prefix = "konsole --hold -p tabtitle='SM Atomic Lifecycle' -e"

    # Construct logging prefix for lifecycle example node
    if log_dir:
        lifecycle_log = os.path.join(log_dir, f"lifecycle_example_node_{timestamp}.log")
        lifecycle_prefix = f"konsole --hold -p tabtitle='Lifecycle Example' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {lifecycle_log}; exec bash' -- "
    else:
        lifecycle_prefix = "konsole --hold -p tabtitle='Lifecycle Example' -e"

    return LaunchDescription(
        [
            Node(
                package="sm_atomic_lifecycle",
                executable="sm_atomic_lifecycle_node",
                output="screen",
                prefix=state_machine_prefix,
            ),
            Node(
                package="sm_atomic_lifecycle",
                executable="lifecycle_example_node",
                output="screen",
                prefix=lifecycle_prefix,
            ),
        ],
    )
