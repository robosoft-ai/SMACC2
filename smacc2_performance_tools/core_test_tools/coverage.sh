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

# Line, function and branch coverage of the SMACC2 core (the smacc2 package) from the core tests and
# the regression suite. Builds an instrumented copy of the packages in its own build and install
# bases, so the workspace's normal build is not touched.
#
# Usage: coverage.sh <workspace-root> <work-dir>
# The workspace's normal install must be built and sourceable (the instrumented build overlays it).
# Writes <work-dir>/core.info and prints the summary; for a browsable report run
#   genhtml <work-dir>/core.info --branch-coverage --output-directory <work-dir>/html
# Needs lcov 2.0 or later. About 12 minutes.
if [ $# -lt 2 ]; then echo "usage: $0 <workspace-root> <work-dir>"; exit 2; fi
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=$(cd "$1" && pwd) || exit 2
C=$(mkdir -p "$2" && cd "$2" && pwd)
TESTS="sm_coretest_forced_finish_1 sm_coretest_poll_shutdown_1 sm_coretest_signal_key_1 sm_coretest_transition_speed_1 sm_coretest_reactor_order_1 sm_coretest_post_during_exit_1 sm_coretest_timer_handoff_1 sm_coretest_callback_rate_1 sm_coretest_exceptions_1 sm_coretest_action_client_1 sm_coretest_features_1 sm_atomic sm_atomic_lifecycle sm_simple_action_client sm_retry_logic_1 sm_mode_state_behavior_1"
cd "$WS" || exit 2
source install/setup.bash
set -u   # after sourcing: colcon's setup scripts read unset variables
# shellcheck disable=SC2086
colcon --log-base "$C/log" build --build-base "$C/build" --install-base "$C/install" \
  --packages-up-to $TESTS --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage -O0" -DCMAKE_EXE_LINKER_FLAGS="--coverage" -DCMAKE_SHARED_LINKER_FLAGS="--coverage" > "$C/build.log" 2>&1 \
  || { echo "instrumented build failed, see $C/build.log"; exit 1; }
bash "$REPO/.github/scripts/sanitizer_core_tests.sh" plain "$C/install" "$C/gate" | grep -a 'FAIL\|result'
bash "$HERE/regression.sh" "$C/install" "$C/reg" "$HERE/regression_expected.out" | tail -1
# Headers reach the machines through the install tree; map them back to the source tree before extracting.
lcov --capture --directory "$C/build" --output-file "$C/all.info" --rc branch_coverage=1 \
  --substitute "s|^.*/install/smacc2/include/|$REPO/smacc2/include/|" \
  --ignore-errors mismatch,inconsistent,negative,gcov,unused,empty,source > "$C/lcov_capture.log" 2>&1
lcov --extract "$C/all.info" "$REPO/smacc2/*" --output-file "$C/core.info" --rc branch_coverage=1 \
  --ignore-errors inconsistent,unused,empty > "$C/lcov_extract.log" 2>&1
lcov --summary "$C/core.info" --rc branch_coverage=1 --ignore-errors inconsistent 2>&1 | tail -4
