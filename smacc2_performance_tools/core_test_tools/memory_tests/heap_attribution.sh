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

# Heap attribution for a memory test: runs the test's machine twice under heaptrack, for <seconds>
# and for twice that, and compares the two. It names the call sites whose live memory grew between
# the runs, and gives the number of heap allocations per unit of work. Use it when a soak fails, to
# find where; and on the transition path, to see what a transition allocates.
#
# Usage: heap_attribution.sh <install-prefix> <test> [seconds] [out-dir]
# Tests: transition, object_churn, idle_load, action_goal. Defaults: 20 seconds (so runs of 20 and
# 40), ./heap_attribution_out. Needs heaptrack. Each run is ended with SIGKILL so that everything
# still allocated counts as live; heaptrack slows the machine by a quarter to a half.
# Exit status: 0 when both runs were analyzed, 2 otherwise. It is a report, not a pass or fail.
if [ $# -lt 2 ]; then sed -n '17,27p' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
TEST=$2; SECS=${3:-20}; OUT=${4:-heap_attribution_out}
HERE=$(cd "$(dirname "$0")" && pwd)
command -v heaptrack > /dev/null || { echo "heaptrack not found (apt package heaptrack)"; exit 2; }
source "$HERE/test_commands.sh"
memory_test_setup "$TEST" "$INST" || exit 2
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
for run in short long; do
  [ $run = short ] && D=$SECS || D=$((SECS * 2))
  rm -f "$OUT/$run.zst" "$OUT/$run.gz"
  heaptrack -o "$OUT/$run" "${MT_CMD[@]}" > "$OUT/$run.log" 2>&1 &
  HP=$!
  sleep "$D"
  # the kernel keeps 15 characters of a process name
  pkill -KILL -u "$(id -u)" -x "${MT_PROC:0:15}"
  wait $HP 2> /dev/null
  DATA=$(ls "$OUT/$run".zst "$OUT/$run".gz 2> /dev/null | head -1)
  [ -n "$DATA" ] || { echo "heaptrack wrote no data for the $run run, see $OUT/$run.log"; exit 2; }
  eval "DATA_$run=\$DATA"
  heaptrack_print -f "$DATA" -p 0 -a 0 -T 0 -l 0 > "$OUT/$run.summary.txt" 2>&1
  echo "$run run: $D s, $(memory_test_count "$TEST" "$OUT/$run.log") $MT_UNIT"
done
heaptrack_print -f "$DATA_long" -d "$DATA_short" -p 0 -a 0 -T 0 -l 1 > "$OUT/diff_leaks.txt" 2>&1
heaptrack_print -f "$DATA_long" -p 0 -a 0 -T 0 -l 0 -F "$OUT/long_stacks.txt" --flamegraph-cost-type allocations > /dev/null 2>&1
python3 -I "$HERE/heap_summary.py" "$OUT" "$MT_UNIT" \
  "$(memory_test_count "$TEST" "$OUT/short.log")" "$(memory_test_count "$TEST" "$OUT/long.log")"
