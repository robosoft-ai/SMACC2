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

# Run the Panda MoveIt reference machine against the MoveIt infrastructure alone (move_group,
# ros2_control with mock hardware, robot_state_publisher; no RViz, no terminal windows), with the
# machine's binary started directly so its exit status and shutdown are observed. Stops the machine
# with SIGINT when it enters the given state and prints a report.
#
# Usage: panda_run.sh <install-prefix> <out-dir> <end-state> [tsan|asan <overlay-prefix>]
# <end-state> is a state name of the machine, for example StMoveJoints2 or StUndoLastTrajectory.
# Environment: PKG (default sm_panda_cl_moveit2z_cb_inventory), MAX_WAIT seconds (default 600).
# Needs moveit_resources_panda_moveit_config. With tsan or asan the machine binary and its SMACC2
# libraries are taken from the overlay prefix.
# Exit status 0 when the end state was reached and the machine stopped without hanging.
if [ $# -lt 3 ]; then echo "usage: $0 <install-prefix> <out-dir> <end-state> [tsan|asan <overlay-prefix>]"; exit 2; fi
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
INST=$(cd "$1" && pwd) || exit 2
mkdir -p "$2"; OUT=$(cd "$2" && pwd)
END_STATE=$3; MODE=${4:-plain}; OVERLAY_DIR=${5:-}; PKG=${PKG:-sm_panda_cl_moveit2z_cb_inventory}; MAX_WAIT=${MAX_WAIT:-600}
source "$INST/setup.bash"
export RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-47}
# A leftover ros2 daemon or node is a participant every node here would discover: stop the daemon, require an empty domain.
ros2 daemon stop > /dev/null 2>&1; trap 'ros2 daemon stop > /dev/null 2>&1' EXIT
LEFT=$(( $(ros2 node list --no-daemon -a 2>/dev/null | wc -l) - 1 ))   # the list holds this command's own node
[ "$LEFT" -le 0 ] || { echo "FAIL: $LEFT node(s) already running in ROS domain $ROS_DOMAIN_ID"; exit 2; }
# Sanitizer mode: the machine and its SMACC2 libraries come from the overlay, everything else stays uninstrumented.
SUPP=$REPO/.github/scripts; PRE=""; OVERLAY=""; INST_SM=$INST
case $MODE in
  plain) ;;
  asan) export ASAN_OPTIONS="halt_on_error=0:detect_leaks=1" LSAN_OPTIONS="suppressions=$SUPP/lsan.supp:print_suppressions=0" UBSAN_OPTIONS="print_stacktrace=1" ;;
  tsan) PRE="setarch x86_64 -R"; export TSAN_OPTIONS="halt_on_error=0:second_deadlock_stack=1:log_path=$OUT/tsan_report:suppressions=$SUPP/tsan.supp" UBSAN_OPTIONS="print_stacktrace=1" ;;
  *) echo "mode must be tsan or asan, not '$MODE'"; exit 2 ;;
esac
if [ $MODE != plain ]; then
  [ -f "$OVERLAY_DIR/local_setup.bash" ] || { echo "no local_setup.bash in the overlay prefix '$OVERLAY_DIR'"; exit 2; }
  INST_SM=$(cd "$OVERLAY_DIR" && pwd); OVERLAY=$INST_SM/local_setup.bash
fi
BIN=$INST_SM/$PKG/lib/$PKG/${PKG}_node
[ -x "$BIN" ] || { echo "no binary at $BIN"; exit 2; }
LOG=$OUT/sm.log
echo "binary: $BIN"
( [ -n "$OVERLAY" ] && source "$OVERLAY"; ldd "$BIN" ) | grep -E "libsmacc2\.|libcl_|libasan|libtsan" | sed 's/^\s*//; s/ (0x.*//'
IL=$OUT/infra.log
SIMPROCS='[m]ove_group|[r]os2_control_node|[r]obot_state_publisher|[_]node --ros-args'
echo "leftover: $(pgrep -u $USER -c -f "$SIMPROCS")"
export PYTHONDONTWRITEBYTECODE=1   # keeps ros2 launch from leaving a __pycache__ beside the launch file
set -m
( exec ros2 launch "$HERE/panda_infra.launch.py" > $IL 2>&1 ) & LP=$!
for i in $(seq 1 90); do sleep 1; grep -a -q "You can start planning now" $IL && { echo "move_group ready after $i s"; break; }; done
grep -a -q "You can start planning now" $IL || echo "move_group NOT ready after 90 s"
sleep 3
T0=$(date +%s)
( [ -n "$OVERLAY" ] && source "$OVERLAY"; exec $PRE $BIN --ros-args --log-level INFO > $LOG 2>&1 ) & SP=$!
VERDICT=timeout
for i in $(seq 1 $MAX_WAIT); do sleep 1
  grep -a -q "::${END_STATE}\] State OnEntry code finished" $LOG && { VERDICT=reached; break; }
  kill -0 $SP 2>/dev/null || { VERDICT=died; break; }
done
echo "verdict: $VERDICT ($END_STATE) after $(( $(date +%s)-T0 )) s"
sleep 3; HANG=no
if kill -0 $SP 2>/dev/null; then T1=$(date +%s.%N); kill -INT $SP
  for i in $(seq 1 120); do kill -0 $SP 2>/dev/null || break; sleep 1; done
  if kill -0 $SP 2>/dev/null; then HANG=yes; echo "SM HANG: alive 120 s after SIGINT, SIGKILL"; kill -KILL $SP; fi
  wait $SP; echo "SM exit $? after $(python3 -c "print(round($(date +%s.%N)-$T1,1))") s"
else wait $SP; echo "SM exit $? (ended on its own)"; fi
kill -INT $LP; for i in $(seq 1 60); do kill -0 $LP 2>/dev/null || break; sleep 1; done
kill -0 $LP 2>/dev/null && { kill -INT -- -$LP; sleep 10; kill -0 $LP 2>/dev/null && kill -KILL -- -$LP; }
wait $LP 2>/dev/null; echo "infra exit $?"
sleep 1; echo "leftover after: $(pgrep -u $USER -c -f "$SIMPROCS")"
echo "== transitions on a behavior failure: $(grep -a -c 'TRANSITION RULE TRIGGERED.*EvCbFailure' $LOG)"
echo "== states: $(grep -a -o "${PKG}::[A-Za-z0-9_]*\] State OnEntry code finished" $LOG | sed 's/.*::\(.*\)\].*/\1/' | tr '\n' ' ')"
echo "== shutdown: $(grep -a -o 'signal_handler\|Terminating scheduler\|Polling loop exited\|Shutdown complete\|Finishing State Machine\|terminate called\|destroyed with a live\|cannot be slept' $LOG | tr '\n' ' ')"
echo "== ERROR $(grep -a -c '\[ERROR\]' $LOG)  FATAL $(grep -a -c '\[FATAL\]' $LOG)  discards $(grep -a -c 'SCOPED EVENT DISCARDED' $LOG)  DDS teardown errors $(grep -a -c 'cannot publish data\|Failed to delete datareader\|Fail in delete datareader' $LOG)  SEGV $(grep -a -c 'SEGV\|Segmentation' $LOG)"
grep -a '\[ERROR\]' $LOG | sed 's/^\[[^]]*\] //' | sort | uniq -c | sort -rn | head -4 | cut -c1-160
if [ $MODE != plain ]; then
  echo "== sanitizer: ASan errors $(grep -a -c '^==[0-9]*==ERROR: AddressSanitizer' $LOG)  LSan $(grep -a -c 'LeakSanitizer' $LOG)  leak: $(grep -a -o 'SUMMARY: AddressSanitizer: [0-9]* byte(s) leaked in [0-9]* allocation' $LOG)  UBSan $(grep -a -c 'runtime error' $LOG)  TSan warnings $(cat $OUT/tsan_report.* 2>/dev/null | grep -c '^WARNING: ThreadSanitizer')"
  python3 $REPO/.github/scripts/sanitizer_reports.py $LOG $OUT/tsan_report.* 2>/dev/null | tail -3
fi
[ $VERDICT = reached ] && [ $HANG = no ]
