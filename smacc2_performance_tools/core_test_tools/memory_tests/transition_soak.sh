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

# Memory test: a bare transition. Runs sm_coretest_transition_speed_1 at full speed with logging off
# under memory_sampler.sh. It finds memory that grows per transition and is still reachable at exit,
# which LSan cannot report.
#
# Usage: transition_soak.sh <install-prefix> [seconds] [out-dir]
# Defaults: 300 seconds, ./transition_soak_out. Limits and sampling: see memory_sampler.sh.
if [ $# -lt 1 ]; then sed -n '17,22p' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-transition_soak_out}
HERE=$(cd "$(dirname "$0")" && pwd)
source "$INST/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
PKG=$INST/sm_coretest_transition_speed_1
bash "$HERE/memory_sampler.sh" "$OUT" "$SECS" \
  "$PKG/lib/sm_coretest_transition_speed_1/sm_coretest_transition_speed_1_node" --ros-args --log-level FATAL \
  --params-file "$PKG/share/sm_coretest_transition_speed_1/config/sm_coretest_transition_speed_1_config.yaml" \
  -r __node:=sm_coretest_transition_speed_1
RC=$?
echo "round trips: about $(($(grep -c 'Longtime frequency:' "$OUT/node.log") * 100))"
exit $RC
