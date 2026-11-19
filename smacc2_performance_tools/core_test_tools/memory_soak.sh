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

# Memory-growth soak: runs sm_coretest_transition_speed_1 at full speed with logging off, samples
# the process's resident memory, and fails if it keeps growing after a warm-up. It finds memory
# that is still reachable at exit, which LSan cannot report (a container that grows per transition).
#
# Usage: memory_soak.sh <install-prefix> [seconds] [out-dir]
# Defaults: 300 seconds, ./memory_soak_out. Environment: SAMPLE_S (2) seconds between samples,
# WARMUP_S (60) seconds ignored at the start, MAX_GROWTH_KB (4096) allowed growth after the warm-up.
# Exit status: 0 pass, 1 growth over the limit or the node died, 2 usage.
if [ $# -lt 1 ]; then sed -n '17,24p' "$0"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-300}; OUT=${3:-memory_soak_out}
SAMPLE_S=${SAMPLE_S:-2}; WARMUP_S=${WARMUP_S:-60}; MAX_GROWTH_KB=${MAX_GROWTH_KB:-4096}
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
source "$INST/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
# A leftover ros2 daemon or node is a participant every node here would discover: stop the daemon, require an empty domain.
ros2 daemon stop > /dev/null 2>&1; trap 'ros2 daemon stop > /dev/null 2>&1' EXIT
LEFT=$(( $(ros2 node list --no-daemon -a 2>/dev/null | wc -l) - 1 ))   # the list holds this command's own node
[ "$LEFT" -le 0 ] || { echo "FAIL: $LEFT node(s) already running in ROS domain $ROS_DOMAIN_ID"; exit 2; }
PKG=$INST/sm_coretest_transition_speed_1
BIN=$PKG/lib/sm_coretest_transition_speed_1/sm_coretest_transition_speed_1_node
PARAMS=$PKG/share/sm_coretest_transition_speed_1/config/sm_coretest_transition_speed_1_config.yaml
"$BIN" --ros-args --log-level FATAL --params-file "$PARAMS" -r __node:=sm_coretest_transition_speed_1 > "$OUT/node.log" 2>&1 &
PID=$!
echo "seconds,rss_kb" > "$OUT/rss.csv"
START=$(date +%s)
while :; do
  T=$(($(date +%s) - START))
  [ "$T" -ge "$SECS" ] && break
  RSS=$(awk '/^VmRSS:/ {print $2}' "/proc/$PID/status" 2> /dev/null)
  if [ -z "$RSS" ]; then echo "node died after $T s, see $OUT/node.log"; DIED=1; break; fi
  echo "$T,$RSS" >> "$OUT/rss.csv"
  sleep "$SAMPLE_S"
done
kill -INT $PID 2> /dev/null; wait $PID; echo "node exit $?"
python3 -I - "$OUT/rss.csv" "$OUT/node.log" "$WARMUP_S" "$MAX_GROWTH_KB" "${DIED:-0}" <<'PY'
import re, sys, statistics as st
rows = [tuple(map(int, line.split(","))) for line in open(sys.argv[1]).read().split()[1:]]
warmup, limit, died = int(sys.argv[3]), int(sys.argv[4]), sys.argv[5] == "1"
batches = len(re.findall(r"Longtime frequency:", open(sys.argv[2], errors="replace").read()))
body = [(t, kb) for t, kb in rows if t >= warmup]
if len(body) < 10:
    print(f"only {len(body)} samples after the {warmup} s warm-up; run longer"); sys.exit(1)
n = max(3, len(body) // 10)   # compare the medians of the first and last tenth
first, last = st.median(kb for _, kb in body[:n]), st.median(kb for _, kb in body[-n:])
span = body[-1][0] - body[0][0]
growth = last - first
print(f"samples: {len(rows)} ({len(body)} after the {warmup} s warm-up)  round trips: about {batches * 100}")
print(f"resident memory: start {rows[0][1]} kB  after warm-up {first:.0f} kB  end {last:.0f} kB  peak {max(kb for _, kb in rows)} kB")
print(f"growth after warm-up: {growth:+.0f} kB over {span} s ({growth * 60 / span:+.1f} kB/min)  limit {limit} kB")
ok = growth <= limit and not died
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
PY
