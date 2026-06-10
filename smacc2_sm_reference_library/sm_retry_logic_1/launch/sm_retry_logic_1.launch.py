# Copyright 2025 Robosoft Inc.
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
    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    log_dir = os.path.join(os.path.expanduser("~"), ".ros", "log", f"{timestamp}-sm_retry_logic_1")
    try:
        os.makedirs(log_dir, mode=0o755, exist_ok=True)
        return log_dir, timestamp
    except (PermissionError, OSError):
        fallback_dir = os.path.join("/tmp", "sm_retry_logic_1_logs", timestamp)
        try:
            os.makedirs(fallback_dir, mode=0o755, exist_ok=True)
            return fallback_dir, timestamp
        except Exception:
            return None, timestamp


def generate_launch_description():
    log_dir, timestamp = setup_log_directory()

    if log_dir:
        sm_log = os.path.join(log_dir, f"state_machine_{timestamp}.log")
        sm_prefix = f"konsole --hold -p tabtitle='SM Retry Logic 1' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {sm_log}; exec bash' -- "
        kb_log = os.path.join(log_dir, f"keyboard_server_{timestamp}.log")
        kb_prefix = f"konsole --hold -p tabtitle='Keyboard Server' -e bash -c 'RCUTILS_COLORIZED_OUTPUT=1 \"$@\" 2>&1 | tee {kb_log}; exec bash' -- "
    else:
        sm_prefix = "konsole --hold -p tabtitle='SM Retry Logic 1' -e"
        kb_prefix = "konsole --hold -p tabtitle='Keyboard Server' -e"

    return LaunchDescription(
        [
            Node(
                package="sm_retry_logic_1",
                executable="sm_retry_logic_1_node",
                name="sm_retry_logic_1",
                output="screen",
                prefix=sm_prefix,
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
            Node(
                package="cl_keyboard",
                executable="keyboard_server_node.py",
                name="keyboard_server_node",
                output="screen",
                prefix=kb_prefix,
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ],
    )
