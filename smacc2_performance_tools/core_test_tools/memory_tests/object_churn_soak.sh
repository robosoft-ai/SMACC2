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

# Memory test: state-scoped objects. Runs sm_coretest_object_churn_1 with logging off under
# memory_sampler.sh. Each of its states creates and destroys a synchronous updatable behavior, an
# asynchronous behavior and its thread, an event generator, a state reactor and two signal
# connections, about 1000 round trips a second with the polling loop at 2000 Hz.
#
# Usage: object_churn_soak.sh <install-prefix> [seconds] [out-dir]
# Defaults: 300 seconds, ./object_churn_soak_out. Limits and sampling: see memory_sampler.sh.
if [ $# -lt 1 ]; then grep -m1 '^# Usage' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-object_churn_soak_out}
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/test_commands.sh"
memory_test_setup object_churn "$INST" || exit 2
bash "$HERE/memory_sampler.sh" "$OUT" "$SECS" "${MT_CMD[@]}"
RC=$?
echo "$MT_UNIT: $(memory_test_count object_churn "$OUT/node.log")"
exit $RC
