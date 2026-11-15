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
if [ $# -lt 1 ]; then grep -m1 '^# Usage' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-action_goal_soak_out}
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/test_commands.sh"
memory_test_setup action_goal "$INST" || exit 2
bash "$HERE/memory_sampler.sh" "$OUT" "$SECS" "${MT_CMD[@]}"
RC=$?
echo "$MT_UNIT: $(memory_test_count action_goal "$OUT/node.log")"
# a trap entry in the last second is the interrupt aborting a goal in flight, not a wrong outcome
TRAPS=$(grep -a -c 'TRAP ENTERED' "$OUT/node.log")
[ "$TRAPS" -gt 0 ] && echo "note: StTrap entered $TRAPS time(s); see $OUT/node.log"
exit $RC
