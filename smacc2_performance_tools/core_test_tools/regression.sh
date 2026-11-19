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

# Plain-build regression suite: five reference state machines, one at a time, binaries run directly.
# Prints the state sequence and counters of each; the output is the same on every run of a healthy
# build, so it is compared with regression_expected.out.
#
# Usage: regression.sh <install-prefix> <out-dir> [expected-file]
# The install prefix must hold sm_atomic, sm_atomic_lifecycle, sm_simple_action_client,
# sm_retry_logic_1 and sm_mode_state_behavior_1, and action_tutorials_cpp must be installed.
# Logs go to <out-dir>/reg_<machine>.log, the printed result to <out-dir>/regression.out.
# With an expected file the exit status is 1 when the result differs from it.
# About 3 minutes. No other ROS node may be running in the domain (ROS_DOMAIN_ID, default 45).
if [ $# -lt 2 ]; then echo "usage: $0 <install-prefix> <out-dir> [expected-file]"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
mkdir -p "$2"; OUT=$(cd "$2" && pwd); EXPECTED=${3:-}
source "$INST/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-45} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
# A leftover ros2 daemon or node is a participant every node here would discover: stop the daemon, require an empty domain.
ros2 daemon stop > /dev/null 2>&1; trap 'ros2 daemon stop > /dev/null 2>&1' EXIT
LEFT=$(( $(ros2 node list --no-daemon -a 2>/dev/null | wc -l) - 1 ))   # the list holds this command's own node
[ "$LEFT" -le 0 ] || { echo "FAIL: $LEFT node(s) already running in ROS domain $ROS_DOMAIN_ID"; exit 2; }
keystream() { timeout $2 ros2 topic pub -r 1 /keyboard_unicode std_msgs/msg/UInt16 "data: $1" > /dev/null 2>&1; }
# Scripted keypresses come from one publisher (send_keys.py), matched and settled before the first key.
SEND_KEYS=$HERE/send_keys.py
nodes_clean() { n=$(ros2 node list --no-daemon 2>/dev/null | wc -l); echo "ros2 node list entries: $n"; }
node_bin() { echo "$INST/$1/lib/$1/$1_node"; }
run_sm() { # name binary timeout_s  (sets P)
  LOG=$OUT/reg_$1.log; timeout -s INT -k 20 $3 $2 --ros-args --log-level INFO > $LOG 2>&1 & P=$!; }

suite() {
echo "######## sm_atomic"; nodes_clean
run_sm sm_atomic $(node_bin sm_atomic) 14; wait $P; echo "exit $?"
echo "state entries: $(grep -a -c 'State OnEntry code finished' $OUT/reg_sm_atomic.log)  State1: $(grep -a -c '::State1\] State OnEntry code finished' $OUT/reg_sm_atomic.log)  State2: $(grep -a -c '::State2\] State OnEntry code finished' $OUT/reg_sm_atomic.log)  'No signals found' at INFO: $(grep -a -c 'No signals found' $OUT/reg_sm_atomic.log)  exceptions: $(grep -a -ci 'exception' $OUT/reg_sm_atomic.log)"
sleep 2

echo "######## sm_atomic_lifecycle"; nodes_clean
LLOG=$OUT/reg_lifecycle_example.log
$INST/sm_atomic_lifecycle/lib/sm_atomic_lifecycle/lifecycle_example_node --ros-args --log-level INFO > $LLOG 2>&1 & LP=$!
sleep 2
run_sm sm_atomic_lifecycle $(node_bin sm_atomic_lifecycle) 70
# The machine drives the node to Active on its own (configure, activate) and stays there by
# design; a shutdown request from outside takes it through StShuttingDown to StFinalized.
for i in $(seq 1 60); do sleep 0.5; grep -a -q '::StActive\] State OnEntry code finished' $OUT/reg_sm_atomic_lifecycle.log && break; kill -0 $P 2>/dev/null || break; done
echo "StActive reached: $(grep -a -c '::StActive\] State OnEntry code finished' $OUT/reg_sm_atomic_lifecycle.log)"
sleep 1; timeout 20 ros2 lifecycle set --no-daemon /lifecycle_example_node shutdown > /dev/null 2>&1
for i in $(seq 1 40); do sleep 0.5; grep -a -q '::StFinalized\] State OnEntry code finished' $OUT/reg_sm_atomic_lifecycle.log && break; kill -0 $P 2>/dev/null || break; done
sleep 2; kill -INT $P 2>/dev/null; wait $P; echo "sm exit $?"; kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null
echo "states entered: $(grep -a 'State OnEntry code finished' $OUT/reg_sm_atomic_lifecycle.log | sed 's/.*::\([A-Za-z0-9_]*\)\].*/\1/' | tr '\n' ' ')"
echo "StFinalized reached: $(grep -a -c '::StFinalized\] State OnEntry code finished' $OUT/reg_sm_atomic_lifecycle.log)  exceptions: $(grep -a -ci 'exception' $OUT/reg_sm_atomic_lifecycle.log)  client rejections: $(grep -a -c 'rejected by' $OUT/reg_sm_atomic_lifecycle.log)"
sleep 2

echo "######## sm_simple_action_client"; nodes_clean
FLOG=$OUT/reg_fibonacci.log
$(ros2 pkg prefix action_tutorials_cpp)/lib/action_tutorials_cpp/fibonacci_action_server --ros-args --log-level INFO > $FLOG 2>&1 & FP=$!
sleep 2
run_sm sm_simple_action_client $(node_bin sm_simple_action_client) 60
sleep 3
python3 $INST/sm_simple_action_client/lib/sm_simple_action_client/auto_mode_trigger.py --ros-args --log-level INFO > $OUT/reg_trigger.log 2>&1 & TP=$!
for i in $(seq 1 110); do sleep 0.5; grep -a -q '::StState3\] State OnEntry code finished' $OUT/reg_sm_simple_action_client.log && break; kill -0 $P 2>/dev/null || break; done
sleep 2; kill -INT $P 2>/dev/null; wait $P; echo "sm exit $?"; kill -INT $TP $FP 2>/dev/null; wait $TP $FP 2>/dev/null
echo "states entered: $(grep -a 'State OnEntry code finished' $OUT/reg_sm_simple_action_client.log | sed 's/.*::\([A-Za-z0-9_]*\)\].*/\1/' | tr '\n' ' ')"
echo "StState3 reached: $(grep -a -c '::StState3\] State OnEntry code finished' $OUT/reg_sm_simple_action_client.log)  StState2 entries: $(grep -a -c '::StState2\] State OnEntry code finished' $OUT/reg_sm_simple_action_client.log)  EvCbSuccess<CbFibonacci: $(grep -a -c 'TRANSITION RULE TRIGGERED.*EvCbSuccess<.*CbFibonacci' $OUT/reg_sm_simple_action_client.log)"
sleep 2

echo "######## sm_retry_logic_1 (exhaust)"; nodes_clean
run_sm sm_retry_logic_1 $(node_bin sm_retry_logic_1) 70
sleep 3; keystream 110 50 &
for i in $(seq 1 120); do sleep 0.5; grep -a -q '::StExhausted\] State OnEntry code finished' $OUT/reg_sm_retry_logic_1.log && break; kill -0 $P 2>/dev/null || break; done
sleep 2; kill -INT $P 2>/dev/null; wait $P; echo "sm exit $?"; wait
echo "StExhausted reached: $(grep -a -c '::StExhausted\] State OnEntry code finished' $OUT/reg_sm_retry_logic_1.log)  attempts: $(grep -a -c '::StAttempting\] State OnEntry code finished' $OUT/reg_sm_retry_logic_1.log)"
grep -a 'State reactor disposing' $OUT/reg_sm_retry_logic_1.log | sed 's/^.*\]: //' | cat -v
sleep 2
echo "######## sm_retry_logic_1 (success)"; nodes_clean
run_sm sm_retry_logic_1s $(node_bin sm_retry_logic_1) 30
sleep 3; python3 $SEND_KEYS n*2 pause:1 s*4
for i in $(seq 1 40); do sleep 0.5; grep -a -q '::StSuccess\] State OnEntry code finished' $OUT/reg_sm_retry_logic_1s.log && break; kill -0 $P 2>/dev/null || break; done
sleep 2; kill -INT $P 2>/dev/null; wait $P; echo "sm exit $?"
echo "StSuccess reached: $(grep -a -c '::StSuccess\] State OnEntry code finished' $OUT/reg_sm_retry_logic_1s.log)"
sleep 2

echo "######## sm_mode_state_behavior_1"; nodes_clean
run_sm sm_mode_state_behavior_1 $(node_bin sm_mode_state_behavior_1) 45
sleep 3; python3 $SEND_KEYS n*5 f pause:1 n*5 f pause:1 n*5 f pause:1 n*5
kill -INT $P 2>/dev/null; wait $P; echo "sm exit $?"
L=$OUT/reg_sm_mode_state_behavior_1.log
echo "EvKeyPressN transitions: $(grep -a -c 'TRANSITION RULE TRIGGERED.*EvKeyPressN' $L)  EvKeyPressF transitions: $(grep -a -c 'TRANSITION RULE TRIGGERED.*EvKeyPressF' $L)  MsRunning entered: $(grep -a -c 'MsRunning\] Entered' $L)  'No signals found': $(grep -a -c 'No signals found' $L)  SsPhase1 entries: $(grep -a -c '::SsPhase1\] State OnEntry code finished' $L)  SsPhase2 entries: $(grep -a -c '::SsPhase2\] State OnEntry code finished' $L)"
echo "states entered: $(grep -a 'State OnEntry code finished' $L | sed 's/.*::\([A-Za-z0-9_]*\)\].*/\1/' | tr '\n' ' ')"
nodes_clean
}

suite 2>&1 | tee "$OUT/regression.out"
if [ -n "$EXPECTED" ]; then
  if diff "$EXPECTED" "$OUT/regression.out" > "$OUT/regression.diff"; then
    echo "regression: identical to $(basename "$EXPECTED")"
  else
    echo "regression: DIFFERS from $(basename "$EXPECTED")"; cat "$OUT/regression.diff"; exit 1
  fi
fi
