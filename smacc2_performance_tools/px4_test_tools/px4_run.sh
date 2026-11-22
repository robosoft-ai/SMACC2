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

# Run a PX4 multirotor reference machine against a headless PX4 SITL (Gazebo x500) and a micro-ROS
# agent, with the machine's binary started directly so its exit status and shutdown are observed.
# Stops the machine with SIGINT when it reaches its end state and prints a report.
#
# Usage: px4_run.sh <install-prefix> <out-dir> <px4-build-dir> [tsan|asan <overlay-prefix>]
# <px4-build-dir> is PX4-Autopilot/build/px4_sitl_default, already built (make px4_sitl).
# Environment: PKG (default sm_cl_px4_mr_test_1), END_STATE (default StLanded), MAX_WAIT seconds (default 420).
# OWN_AGENT=1 starts no agent here, for a machine that launches its own through CpMicroRosAgent; the
# report then says how many agent processes the machine left behind.
# PX4 arms only with a ground station connected: start QGroundControl first.
# CTRL_C=1 starts the machine through its own launch file, as a user does, and stops it with one
# SIGINT to the launch's process group, as Ctrl-C in its terminal does. PX4 and the agent, which a
# user runs in other terminals, are stopped afterwards as usual.
# With tsan or asan the machine binary and its SMACC2 libraries are taken from the overlay prefix.
# Exit status 0 when the end state was reached and the machine stopped without hanging.
# Runs in ROS domain 0, the domain PX4's DDS client uses.
if [ $# -lt 3 ]; then echo "usage: $0 <install-prefix> <out-dir> <px4-build-dir> [tsan|asan <overlay-prefix>]"; exit 2; fi
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
INST=$(cd "$1" && pwd) || exit 2
mkdir -p "$2"; OUT=$(cd "$2" && pwd)
PX4=$(cd "$3" && pwd) || exit 2
[ -x "$PX4/bin/px4" ] || { echo "no bin/px4 in $PX4"; exit 2; }
MODE=${4:-plain}; OVERLAY_DIR=${5:-}; PKG=${PKG:-sm_cl_px4_mr_test_1}; END_STATE=${END_STATE:-StLanded}; MAX_WAIT=${MAX_WAIT:-420}
source "$INST/setup.bash"
export RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0 ROS_DOMAIN_ID=0
# A leftover ros2 daemon or node is a participant every node here would discover; with OWN_AGENT=1 the agent is one, so no check.
ros2 daemon stop > /dev/null 2>&1; trap 'ros2 daemon stop > /dev/null 2>&1' EXIT
LEFT=$(( $(ros2 node list --no-daemon -a 2>/dev/null | wc -l) - 1 ))   # the list holds this command's own node
[ "$LEFT" -le 0 ] || [ "${OWN_AGENT:-0}" = 1 ] || { echo "FAIL: $LEFT node(s) already running in ROS domain $ROS_DOMAIN_ID"; exit 2; }
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
PL=$OUT/px4.log; AL=$OUT/agent.log
SIMPROCS='[g]z sim|bin/[p]x4|[m]icro_ros_agent|[_]node --ros-args'
echo "leftover: $(pgrep -u $USER -c -f "$SIMPROCS")"
pgrep -u $USER -f '[Q]GroundControl' > /dev/null || echo "WARNING: QGroundControl is not running; PX4 will refuse to arm"
set -m
FIFO=$OUT/px4_stdin; rm -f $FIFO; mkfifo $FIFO   # px4 reads its shell from stdin; "shutdown" goes in here at the end
( cd $PX4/rootfs && exec env HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 $PX4/bin/px4 < $FIFO > $PL 2>&1 ) & PP=$!
exec 3> $FIFO
for i in $(seq 1 90); do sleep 1; grep -a -q "Ready for takeoff\|pxh>" $PL && { echo "px4 booted after $i s"; break; }; done
AP=""
if [ "${OWN_AGENT:-0}" != 1 ]; then
  ( exec ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 > $AL 2>&1 ) & AP=$!
  for i in $(seq 1 60); do sleep 1; grep -a -q "create_topic\|create_participant" $AL && { echo "agent has a client after $i s"; break; }; done
fi
sleep 3
T0=$(date +%s); CTRL_C=${CTRL_C:-0}
if [ $CTRL_C = 1 ]; then
  # env --default-signal: a caller that runs this script as a background job leaves SIGINT ignored, and ros2 launch would then never stop.
  LOG=$OUT/launch.log
  ( [ -n "$OVERLAY" ] && source "$OVERLAY"; exec env --default-signal=INT $PRE ros2 launch $PKG $PKG.launch.py > $LOG 2>&1 ) & SP=$!
else
  ( [ -n "$OVERLAY" ] && source "$OVERLAY"; exec $PRE $BIN --ros-args --log-level INFO > $LOG 2>&1 ) & SP=$!
fi
VERDICT=timeout
for i in $(seq 1 $MAX_WAIT); do sleep 1
  grep -a -q "${END_STATE}\] State OnEntry code finished" $LOG && { VERDICT=reached; break; }
  kill -0 $SP 2>/dev/null || { VERDICT=died; break; }
done
echo "verdict: $VERDICT ($END_STATE) after $(( $(date +%s)-T0 )) s"
sleep 3; HANG=no; SMX=0
if [ $CTRL_C = 1 ]; then
  SMP=$(pgrep -u $USER -f "^[^ ]*${PKG}_node" | head -1); T1=$(date +%s.%N)
  kill -INT -- -$SP   # the process group: ros2 launch and the machine
  for i in $(seq 1 300); do kill -0 "$SMP" 2>/dev/null || break; sleep 0.1; done
  TS=$(python3 -c "print(round($(date +%s.%N)-$T1,1))")
  for i in $(seq 1 60); do kill -0 $SP 2>/dev/null || break; sleep 1; done
  kill -0 $SP 2>/dev/null && { echo "launch alive after 60 s; SIGKILL to the group"; kill -KILL -- -$SP; }
  wait $SP 2>/dev/null; LX=$?
  # ros2 launch reports how the machine ended; the machine's own lines become sm.log
  SMX=$(grep -a "\[${PKG}_node-[0-9]*\]: process has" $LOG | tail -1 | sed -n 's/.*finished cleanly.*/0/p; s/.*exit code \(-\?[0-9]*\).*/\1/p'); SMX=${SMX:-unknown}
  grep -a -q "\[${PKG}_node-[0-9]*\]: process.* failed to terminate" $LOG && { HANG=yes; echo "SM HANG: ros2 launch had to escalate past SIGINT"; }
  echo "SM exit $SMX after $TS s"; echo "launch exit $LX, interrupted $(grep -a -c 'user interrupted with ctrl-c' $LOG)"
  LL=$LOG; LOG=$OUT/sm.log; grep -a "^\[${PKG}_node-[0-9]*\] " $LL | sed 's/^\[[^]]*\] //' > $LOG
elif kill -0 $SP 2>/dev/null; then T1=$(date +%s.%N); kill -INT $SP
  for i in $(seq 1 120); do kill -0 $SP 2>/dev/null || break; sleep 1; done
  if kill -0 $SP 2>/dev/null; then HANG=yes; echo "SM HANG: alive 120 s after SIGINT, SIGKILL"; kill -KILL $SP; fi
  wait $SP; echo "SM exit $? after $(python3 -c "print(round($(date +%s.%N)-$T1,1))") s"
else wait $SP; echo "SM exit $? (ended on its own)"; fi
[ -z "$AP" ] && echo "agent processes left by the machine: $(pgrep -u $USER -c -f '[m]icro_ros_agent')"
[ -n "$AP" ] && kill -INT $AP; printf 'shutdown\n' >&3; exec 3>&-
for i in $(seq 1 30); do kill -0 $PP 2>/dev/null || break; sleep 1; done; kill -0 $PP 2>/dev/null && kill -KILL $PP
for p in $(pgrep -u $USER -f '[g]z sim.*default.sdf|[m]icro_ros_agent'); do kill -KILL $p 2>/dev/null; done
[ -n "$AP" ] && wait $AP 2>/dev/null; wait $PP 2>/dev/null; rm -f $FIFO
sleep 1; echo "leftover after: $(pgrep -u $USER -c -f "$SIMPROCS")"
echo "== px4: arming, takeoff and landing lines $(grep -a -c -i 'armed by\|Takeoff detected\|Landing detected' $PL)"
echo "== states: $(grep -a -o "${PKG}::[A-Za-z0-9_]*\] State OnEntry code finished" $LOG | sed 's/.*::\(.*\)\].*/\1/' | tr '\n' ' ')"
echo "== shutdown: $(grep -a -o 'signal_handler\|Terminating scheduler\|Polling loop exited\|Shutdown complete\|Finishing State Machine\|terminate called\|destroyed with a live\|cannot be slept' $LOG | tr '\n' ' ')"
echo "== ERROR $(grep -a -c '\[ERROR\]' $LOG)  FATAL $(grep -a -c '\[FATAL\]' $LOG)  discards $(grep -a -c 'SCOPED EVENT DISCARDED' $LOG)  DDS teardown errors $(grep -a -c 'cannot publish data\|Failed to delete datareader\|Fail in delete datareader' $LOG)  SEGV $(grep -a -c 'SEGV\|Segmentation' $LOG)"
grep -a '\[ERROR\]' $LOG | sed 's/^\[[^]]*\] //' | sort | uniq -c | sort -rn | head -4 | cut -c1-160
if [ $MODE != plain ]; then
  echo "== sanitizer: ASan errors $(grep -a -c '^==[0-9]*==ERROR: AddressSanitizer' $LOG)  LSan $(grep -a -c 'LeakSanitizer' $LOG)  leak: $(grep -a -o 'SUMMARY: AddressSanitizer: [0-9]* byte(s) leaked in [0-9]* allocation' $LOG)  UBSan $(grep -a -c 'runtime error' $LOG)  TSan warnings $(cat $OUT/tsan_report.* 2>/dev/null | grep -c '^WARNING: ThreadSanitizer')"
  python3 $REPO/.github/scripts/sanitizer_reports.py $LOG $OUT/tsan_report.* 2>/dev/null | tail -3
fi
[ $VERDICT = reached ] && [ $HANG = no ] && { [ $CTRL_C != 1 ] || [ "$SMX" = 0 ] || [ $MODE != plain ]; }
