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

# Build every SMACC2 package with clang and print every warning the compiler gave.
#
# Usage: clang_build.sh <out-dir>
# Run from the colcon workspace root with only /opt/ros/<distro>/setup.bash sourced. The build, install
# and log bases go under the out dir and are deleted first, so an existing GCC build is not touched and
# every file is compiled. The message packages
# (smacc2_msgs, px4_msgs) are built first with the default compiler: their code is generated.
# Writes console.log and warnings.txt to the out dir. Exits 1 on a build error or on any warning.
# Environment: CLANG_VERSION (default 18).
set -u
if [ $# -lt 1 ]; then sed -n '17,25p' "$0"; exit 2; fi
OUT=$1; V=${CLANG_VERSION:-18}
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
command -v "clang++-$V" > /dev/null || { echo "clang++-$V not found: install clang-$V"; exit 2; }
case ":${AMENT_PREFIX_PATH:-}:" in
  *"/smacc2:"*) echo "a SMACC2 install is sourced in this shell; source only /opt/ros/<distro>/setup.bash"; exit 2 ;;
esac
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
# Always from scratch: a file that is not recompiled gives no warning, and the list would come out short.
rm -rf "$OUT/build" "$OUT/install" "$OUT/log"
COLCON=(colcon --log-base "$OUT/log" build --build-base "$OUT/build" --install-base "$OUT/install" --symlink-install)
CMAKE=(--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo)
# Machines whose one compile needs over 2 GB, built last and two at a time, as in the binary build workflow.
HEAVY="sm_nav2_gazebo_test_2 sm_nav2_gazebo_test_3 sm_nav2_gazebo_test_4 sm_panda_cl_moveit2z_cb_inventory_isaacsim sm_cl_px4_mr_test_4 sm_cl_px4_mr_test_5"
OURS=$(colcon list --names-only --base-paths "$REPO"/smacc2*/ | grep -vx smacc2_msgs)
LIGHT=$(echo "$OURS" | grep -vxF -f <(echo "$HEAVY" | tr ' ' '\n'))
echo "### message packages, default compiler"
"${COLCON[@]}" --packages-select smacc2_msgs px4_msgs "${CMAKE[@]}" > "$OUT/messages.log" 2>&1 \
  || { tail -30 "$OUT/messages.log"; echo "FAIL: the message packages did not build"; exit 1; }
echo "### $(echo "$OURS" | wc -l) SMACC2 packages, $("clang++-$V" --version | head -1)"
export CC=clang-$V CXX=clang++-$V
STATUS=0
# shellcheck disable=SC2086
{
  "${COLCON[@]}" --continue-on-error --event-handlers console_cohesion+ --packages-select $LIGHT "${CMAKE[@]}" || STATUS=1
  "${COLCON[@]}" --continue-on-error --event-handlers console_cohesion+ --parallel-workers 2 --packages-select $HEAVY \
    "${CMAKE[@]}" || STATUS=1
} > "$OUT/console.log" 2>&1
grep -E '^(Summary|  [0-9]+ packages? (failed|aborted|had stderr|not processed))' "$OUT/console.log"
python3 "$HERE/clang_warnings.py" "$OUT/console.log" | tee "$OUT/warnings.txt"
[ "$STATUS" -eq 0 ] || { echo "FAIL: the clang build did not complete"; exit 1; }
head -1 "$OUT/warnings.txt" | grep -q '^clang build: 0 errors, 0 warnings' || exit 1
