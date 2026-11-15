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

# Memory test: the polling loop of a machine that stays in one state. Runs sm_coretest_idle_load_1
# with logging off under memory_sampler.sh: a 500 Hz timer, a 1 kB message published and received on
# each tick, an event posted per message that no state consumes, and two updatables, with the
# polling loop at 2000 Hz. It finds growth per loop iteration, per timer tick or per message.
#
# Usage: idle_load_soak.sh <install-prefix> [seconds] [out-dir]
# Defaults: 300 seconds, ./idle_load_soak_out. Limits and sampling: see memory_sampler.sh.
# SM_CORETEST_IDLE_LEAK=1 makes the machine keep every message; the run must then fail.
if [ $# -lt 1 ]; then sed -n '17,25p' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-idle_load_soak_out}
HERE=$(cd "$(dirname "$0")" && pwd)
source "$INST/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
bash "$HERE/memory_sampler.sh" "$OUT" "$SECS" \
  "$INST/sm_coretest_idle_load_1/lib/sm_coretest_idle_load_1/sm_coretest_idle_load_1_node" \
  --ros-args --log-level FATAL -p signal_detector_loop_freq:=2000.0
RC=$?
echo "last count: $(grep -o 'messages: [0-9]* updates: [0-9]*' "$OUT/node.log" | tail -1)"
exit $RC
