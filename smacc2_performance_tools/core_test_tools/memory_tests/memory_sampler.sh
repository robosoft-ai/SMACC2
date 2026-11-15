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

# Shared sampler for the memory tests: runs one command, samples its resident memory, thread count
# and open file descriptors, and fails if any of them keeps growing after a warm-up.
#
# Usage: memory_sampler.sh <out-dir> <seconds> <command> [args...]
# Environment: SAMPLE_S (2) seconds between samples, WARMUP_S (60) seconds ignored at the start,
# MAX_GROWTH_KB (4096), MAX_THREAD_GROWTH (2) and MAX_FD_GROWTH (2) allowed growth after the warm-up.
# The command is stopped with SIGINT; one that does not exit within 20 s fails the run. Its output goes to <out-dir>/node.log, the samples to
# <out-dir>/samples.csv. Exit status: 0 pass, 1 growth over a limit, or the command died or would not stop, 2 usage.
if [ $# -lt 3 ]; then sed -n '17,24p' "$0"; exit 2; fi
OUT=$1; SECS=$2; shift 2
SAMPLE_S=${SAMPLE_S:-2}; WARMUP_S=${WARMUP_S:-60}
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
"$@" > "$OUT/node.log" 2>&1 &
PID=$!
echo "seconds,rss_kb,threads,fds" > "$OUT/samples.csv"
START=$(date +%s)
while :; do
  T=$(($(date +%s) - START))
  [ "$T" -ge "$SECS" ] && break
  RSS=$(awk '/^VmRSS:/ {r=$2} /^Threads:/ {t=$2} END {if (r) print r "," t}' "/proc/$PID/status" 2> /dev/null)
  if [ -z "$RSS" ]; then echo "the command died after $T s, see $OUT/node.log"; DIED=1; break; fi
  echo "$T,$RSS,$(ls "/proc/$PID/fd" 2> /dev/null | wc -l)" >> "$OUT/samples.csv"
  sleep "$SAMPLE_S"
done
kill -INT $PID 2> /dev/null
# a command that ignores SIGINT, or hangs on it, gets SIGTERM after 20 s and SIGKILL after 30 s
for i in $(seq 60); do
  kill -0 $PID 2> /dev/null || break
  [ "$i" = 40 ] && { echo "no exit 20 s after SIGINT, sending SIGTERM"; kill -TERM $PID; STUCK=1; }
  [ "$i" = 60 ] && { echo "no exit 30 s after SIGINT, sending SIGKILL"; kill -KILL $PID; }
  sleep 0.5
done
wait $PID; echo "exit status of the command: $?"
python3 -I - "$OUT/samples.csv" "$WARMUP_S" "${DIED:-${STUCK:-0}}" "${MAX_GROWTH_KB:-4096}" "${MAX_THREAD_GROWTH:-2}" "${MAX_FD_GROWTH:-2}" <<'PY'
import sys, statistics as st
rows = [tuple(map(int, line.split(","))) for line in open(sys.argv[1]).read().split()[1:]]
warmup, died = int(sys.argv[2]), sys.argv[3] == "1"
body = [r for r in rows if r[0] >= warmup]
if len(body) < 10:
    print(f"only {len(body)} samples after the {warmup} s warm-up; run longer"); sys.exit(1)
n = max(3, len(body) // 10)   # compare the medians of the first and last tenth
span = body[-1][0] - body[0][0]
print(f"samples: {len(rows)} ({len(body)} after the {warmup} s warm-up, {span} s)")
ok = not died
for col, (name, unit) in enumerate([("resident memory", " kB"), ("threads", ""), ("file descriptors", "")], 1):
    limit = int(sys.argv[3 + col])
    first, last = st.median(r[col] for r in body[:n]), st.median(r[col] for r in body[-n:])
    growth = last - first
    verdict = "ok" if growth <= limit else "OVER THE LIMIT"
    ok = ok and growth <= limit
    print(f"{name}: start {rows[0][col]}{unit}  after warm-up {first:.0f}{unit}  end {last:.0f}{unit}  "
          f"peak {max(r[col] for r in rows)}{unit}  growth {growth:+.0f}{unit} (limit {limit}{unit})  {verdict}")
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
PY
