#!/bin/bash

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

# Sourced by the memory tests: the command line, environment and work counter of each test machine,
# kept in one place so that the soaks and heap_attribution.sh run exactly the same thing.
#
#   memory_test_setup <test> <install-prefix>   sets MT_CMD (array), MT_UNIT, MT_PROC and the environment
#   memory_test_count <test> <node-log>         prints how many units of work the log records
# Tests: transition, object_churn, idle_load, action_goal.
MEMORY_TESTS="transition object_churn idle_load action_goal"

memory_test_setup() {
  local inst=$2 pkg
  source "$inst/setup.bash"
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
  case $1 in
    transition)
      pkg=sm_coretest_transition_speed_1; MT_UNIT="round trips"
      MT_ARGS=(--params-file "$inst/$pkg/share/$pkg/config/${pkg}_config.yaml" -r "__node:=$pkg") ;;
    object_churn)
      pkg=sm_coretest_object_churn_1; MT_UNIT="round trips"
      MT_ARGS=(-p signal_detector_loop_freq:=2000.0) ;;
    idle_load)
      pkg=sm_coretest_idle_load_1; MT_UNIT="messages"
      MT_ARGS=(-p signal_detector_loop_freq:=2000.0) ;;
    action_goal)
      pkg=sm_coretest_action_client_1; MT_UNIT="goals"
      MT_ARGS=(-p signal_detector_loop_freq:=2000.0)
      export SM_CORETEST_ACTION_STEP_MS=${SM_CORETEST_ACTION_STEP_MS:-2}
      export SM_CORETEST_ACTION_CANCEL_MS=${SM_CORETEST_ACTION_CANCEL_MS:-20} ;;
    *) echo "unknown test '$1'; one of: $MEMORY_TESTS"; return 2 ;;
  esac
  MT_PROC=${pkg}_node
  MT_CMD=("$inst/$pkg/lib/$pkg/${pkg}_node" --ros-args --log-level FATAL "${MT_ARGS[@]}")
}

memory_test_count() {
  local n
  case $1 in
    transition) echo $(($(grep -a -c 'Longtime frequency:' "$2") * 100)) ;;   # one line per 100
    object_churn) n=$(grep -a -o 'round trips: [0-9]*' "$2" | tail -1); echo "${n##* }" ;;
    idle_load) n=$(grep -a -o 'messages: [0-9]*' "$2" | tail -1); echo "${n##* }" ;;
    action_goal) n=$(grep -a -o 'cycles: [0-9]*' "$2" | tail -1); echo $((${n##* } * 5)) ;;   # five goals a cycle
  esac
}
