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

# Run the four core test state machines under a sanitizer build and fail on any report with a
# SMACC2 frame, a crash, a hang or a broken test invariant.
#
# Usage: sanitizer_core_tests.sh <tsan|asan> [install-prefix] [out-dir]
# The install prefix must hold smacc2, cl_ros2_timer, sr_event_countdown and the four core tests
# built with the matching -fsanitize flags. Logs and report files go to the out dir.
MODE=${1:-}; INST=${2:-install}; OUT=${3:-sanitizer_out}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
source "$INST/setup.bash"   # before set -u: colcon's setup scripts read unset variables
set -u
export RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export UBSAN_OPTIONS="print_stacktrace=1"
case $MODE in
  tsan)
    # Current kernels randomize mmap with 32 bits, which TSan's shadow layout cannot take; the run
    # needs ASLR off (setarch -R), and that needs the container's seccomp profile to allow personality().
    PRE="setarch x86_64 -R"
    export TSAN_OPTIONS="halt_on_error=0:second_deadlock_stack=1:log_path=$OUT/tsan_report:suppressions=$HERE/tsan.supp"
    REPORT_OK_EXIT=66 ;;   # TSan exits 66 when it printed a report; the classifier decides
  asan)
    PRE=""
    export ASAN_OPTIONS="halt_on_error=0:detect_leaks=1"
    export LSAN_OPTIONS="suppressions=$HERE/lsan.supp:print_suppressions=0"
    REPORT_OK_EXIT=23 ;;   # LSan exits 23 when it printed a leak; the classifier decides
  *) echo "usage: $0 <tsan|asan> [install-prefix] [out-dir]"; exit 2 ;;
esac
FAILS=0
fail() { echo "FAIL: $*"; FAILS=$((FAILS + 1)); }
bin() { echo "$INST/$1/lib/$1/$1_node"; }
# wait_pattern <log> <pattern> <tenths> : poll the log until the pattern appears
wait_pattern() { for _ in $(seq 1 "$3"); do grep -a -q "$2" "$1" && return 0; sleep 0.1; done; return 1; }
# stop <pid> <log> <label> : SIGINT, wait up to 20 s, SIGKILL on a hang; checks the exit status
stop() {
  local pid=$1 log=$2 label=$3 hang=no status
  kill -INT "$pid" 2>/dev/null
  for _ in $(seq 1 200); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  if kill -0 "$pid" 2>/dev/null; then hang=yes; kill -KILL "$pid" 2>/dev/null; fi
  wait "$pid" 2>/dev/null; status=$?
  [ $hang = yes ] && fail "$label: still running 20 s after SIGINT"
  grep -a -q "terminate called" "$log" && fail "$label: uncaught exception (terminate called)"
  if [ $status -ne 0 ] && [ $status -ne "$REPORT_OK_EXIT" ]; then fail "$label: exit status $status"; fi
  echo "$label: exit $status hang $hang"
}

echo "### forced finish: 20 cycles, SIGINT in StRest"
LOG=$OUT/forced_finish_cycles.log
$PRE "$(bin sm_coretest_forced_finish_1)" --ros-args --log-level INFO > "$LOG" 2>&1 & P=$!
for _ in $(seq 1 600); do
  sleep 0.5
  [ "$(grep -a -c 'StRest\] Entered' "$LOG")" -gt 20 ] && break
  kill -0 $P 2>/dev/null || break
done
stop $P "$LOG" forced_finish_cycles
CYC=$(grep -a -c 'StRest\] Entered' "$LOG"); DIS=$(grep -a -c 'SCOPED EVENT DISCARDED' "$LOG"); TRAP=$(grep -a -c 'TRAP ENTERED' "$LOG")
echo "cycles $CYC discards $DIS traps $TRAP"
[ "$CYC" -ge 20 ] || fail "forced_finish_cycles: only $CYC cycles"
[ "$DIS" -eq $((2 * CYC)) ] || fail "forced_finish_cycles: $DIS discards for $CYC cycles, expected $((2 * CYC))"
[ "$TRAP" -eq 0 ] || fail "forced_finish_cycles: trap state entered $TRAP times"

echo "### signal key: 14 s"
LOG=$OUT/signal_key.log
$PRE "$(bin sm_coretest_signal_key_1)" --ros-args --log-level INFO > "$LOG" 2>&1 & P=$!
sleep 14; stop $P "$LOG" signal_key
DEAD=$(python3 -I - "$LOG" <<'PY'
import sys
in_a = None; behavior = 0; state = 0
for line in open(sys.argv[1], errors="replace"):
    if "[StA] Entered" in line: in_a = True
    elif "[StRest] Entered" in line: in_a = False
    elif in_a is False and "STATE TICK" in line: state += 1
    elif in_a is False and "] TICK" in line: behavior += 1
print(f"{behavior} {state}")
PY
)
echo "ticks into destroyed objects (behavior state): $DEAD"
[ "$DEAD" = "0 0" ] || fail "signal_key: callbacks reached destroyed objects ($DEAD)"
[ "$(grep -a -c 'StA\] Entered' "$LOG")" -ge 5 ] || fail "signal_key: fewer than 5 StA lives"

# sweep <pkg> <label> <runs> [extra ros args]: SIGINT at a random 0.2 to 4.2 s after the first state entry
sweep() {
  local pkg=$1 label=$2 runs=$3; shift 3
  for n in $(seq 1 "$runs"); do
    local log=$OUT/${label}_$n.log off
    off=$(awk -v r=$RANDOM 'BEGIN{printf "%.2f", 0.2 + 4.0 * r / 32767}')
    $PRE "$(bin "$pkg")" --ros-args --log-level INFO "$@" > "$log" 2>&1 & local p=$!
    wait_pattern "$log" 'State OnEntry code finished' 150 || true
    sleep "$off"
    printf "%s run %s offset %s s: " "$label" "$n" "$off"
    stop $p "$log" "${label}_$n"
  done
}
echo "### poll shutdown: 10 interrupts at the 75 % window (5 Hz loop, 150 ms update)"
sweep sm_coretest_poll_shutdown_1 poll_shutdown 10 -p signal_detector_loop_freq:=5.0
echo "### forced finish: 10 random interrupts"
sweep sm_coretest_forced_finish_1 forced_finish_sweep 10

echo "### transition speed: 20 s at 2000 Hz"
LOG=$OUT/transition_speed.log
SPEED=$INST/sm_coretest_transition_speed_1
$PRE "$(bin sm_coretest_transition_speed_1)" --ros-args --log-level FATAL \
  --params-file "$SPEED/share/sm_coretest_transition_speed_1/config/sm_coretest_transition_speed_1_config.yaml" > "$LOG" 2>&1 & P=$!
sleep 20; stop $P "$LOG" transition_speed
echo "batches $(grep -a -c 'Longtime frequency' "$LOG")"
[ "$(grep -a -c 'Longtime frequency' "$LOG")" -gt 0 ] || fail "transition_speed: no batch completed"

echo "### sanitizer reports"
python3 -I "$HERE/sanitizer_reports.py" "$OUT"/*.log "$OUT"/tsan_report.* 2>/dev/null || FAILS=$((FAILS + 1))
echo "### result: $FAILS failure(s)"
[ "$FAILS" -eq 0 ]
