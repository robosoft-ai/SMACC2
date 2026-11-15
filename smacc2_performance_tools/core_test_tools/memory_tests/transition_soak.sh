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
if [ $# -lt 1 ]; then grep -m1 '^# Usage' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-transition_soak_out}
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/test_commands.sh"
memory_test_setup transition "$INST" || exit 2
bash "$HERE/memory_sampler.sh" "$OUT" "$SECS" "${MT_CMD[@]}"
RC=$?
echo "$MT_UNIT: $(memory_test_count transition "$OUT/node.log")"
exit $RC
