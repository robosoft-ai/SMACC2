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

# Memory test: action goals. Runs sm_coretest_action_client_1 with logging off under
# memory_sampler.sh, with its server stepping every 2 ms and its cancel timer at 20 ms, so that the
# five goal outcomes (succeeded, rejected, aborted, no server, cancelled by a state exit) go through
# CpActionClient about 27 times a second each. It finds growth per goal: handles, result callbacks
# and signal connections that are kept after the goal has ended.
#
# Usage: action_goal_soak.sh <install-prefix> [seconds] [out-dir]
# Defaults: 300 seconds, ./action_goal_soak_out. Limits and sampling: see memory_sampler.sh.
if [ $# -lt 1 ]; then sed -n '17,25p' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-action_goal_soak_out}
HERE=$(cd "$(dirname "$0")" && pwd)
source "$INST/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
export SM_CORETEST_ACTION_STEP_MS=${SM_CORETEST_ACTION_STEP_MS:-2} SM_CORETEST_ACTION_CANCEL_MS=${SM_CORETEST_ACTION_CANCEL_MS:-20}
bash "$HERE/memory_sampler.sh" "$OUT" "$SECS" \
  "$INST/sm_coretest_action_client_1/lib/sm_coretest_action_client_1/sm_coretest_action_client_1_node" \
  --ros-args --log-level FATAL -p signal_detector_loop_freq:=2000.0
RC=$?
CYCLES=$(grep -o 'cycles: [0-9]*' "$OUT/node.log" | tail -1 | grep -o '[0-9]*$')
echo "cycles: ${CYCLES:-0} (five goals each)"
# a trap entry in the last second is the interrupt aborting a goal in flight, not a wrong outcome
TRAPS=$(grep -c 'TRAP ENTERED' "$OUT/node.log")
[ "$TRAPS" -gt 0 ] && echo "note: StTrap entered $TRAPS time(s); see $OUT/node.log"
exit $RC
