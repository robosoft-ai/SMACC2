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

# Profile one SMACC2 node with perf and show where its processor time goes: tables in the terminal,
# folded stacks and a flame graph. Use it to look inside a step that the LTTng trace shows to be slow.
#
# Usage: perf_flame.sh <out-dir> <seconds> <node-binary> [node args...]
# Environment: FREQ samples per second (default 999), STACK bytes of stack copied per sample (default
# 32768), SUMMARY_ARGS options passed to perf_summary.py (for example "--thread busiest").
# Needs perf (linux-tools) and kernel.perf_event_paranoid at 2 or lower. The node should be built with
# debug information (RelWithDebInfo); no frame pointers are needed.
set -u
if [ $# -lt 3 ]; then sed -n '17,24p' "$0"; exit 2; fi
OUT=$1; SECS=$2; shift 2
HERE=$(cd "$(dirname "$0")" && pwd); mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
command -v perf > /dev/null || { echo "perf not found: install linux-tools-$(uname -r)"; exit 2; }
LEVEL=$(cat /proc/sys/kernel/perf_event_paranoid)
if [ "$LEVEL" -gt 2 ]; then
  echo "kernel.perf_event_paranoid is $LEVEL, which forbids profiling. Run, once per boot:"
  echo "  sudo sysctl kernel.perf_event_paranoid=1"
  exit 2
fi
# cpu-clock makes each sample's period processor time in nanoseconds, on any machine
perf record -o "$OUT/perf.data" -e cpu-clock -F "${FREQ:-999}" --call-graph "dwarf,${STACK:-32768}" -- \
  bash -c 'log=$1; shift; exec timeout -s INT "$@" > "$log" 2>&1' _ "$OUT/node.log" "$SECS" "$@" 2> "$OUT/perf.err"
echo "node exit $?"
tail -1 "$OUT/perf.err"
# One perf script pass per thread: some perf builds lose the stacks of every thread but the first in one pass.
: > "$OUT/script.txt"
for TID in $(perf script -i "$OUT/perf.data" -G -F tid 2> /dev/null | sort -un); do
  perf script -i "$OUT/perf.data" --tid "$TID" -F comm,tid,period,ip,sym,dso >> "$OUT/script.txt" 2>> "$OUT/perf.err"
done
# shellcheck disable=SC2086
python3 "$HERE/perf_summary.py" "$OUT/script.txt" --out-dir "$OUT" ${SUMMARY_ARGS:-} | tee "$OUT/summary.txt"
