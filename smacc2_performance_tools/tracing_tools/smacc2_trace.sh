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

# Capture an LTTng userspace trace of one SMACC2 node and print the timing summary.
# Usage: smacc2_trace.sh <out-dir> <seconds> <node-binary> [node args...]
# Needs lttng-tools and babeltrace; no root and no tracing group (a per-user session daemon is spawned).
set -u
OUT=$1; SECS=$2; shift 2
HERE=$(cd "$(dirname "$0")" && pwd); mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
S=smacc2_$$
lttng create $S --output="$OUT/trace" > /dev/null || exit 1
lttng enable-channel -u --subbuf-size 8M --num-subbuf 8 ch > /dev/null
lttng enable-event -u -c ch 'ros2:smacc2*' > /dev/null
lttng enable-event -u -c ch ros2:spinOnce > /dev/null
lttng add-context -u -c ch -t vtid -t procname > /dev/null
lttng start > /dev/null
timeout -s INT "$SECS" "$@" > "$OUT/node.log" 2>&1; echo "node exit $?"
lttng stop > /dev/null; lttng destroy $S > /dev/null
babeltrace "$OUT/trace" > "$OUT/trace.txt" 2> "$OUT/babeltrace.err"
echo "trace: $(du -sh "$OUT/trace" | cut -f1), $(wc -l < "$OUT/trace.txt") events, text in $OUT/trace.txt"
python3 "$HERE/trace_summary.py" "$OUT/trace.txt" | tee "$OUT/summary.txt"
