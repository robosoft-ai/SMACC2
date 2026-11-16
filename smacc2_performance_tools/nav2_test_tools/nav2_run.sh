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

# Run one of the Nav2 reference machines (sm_nav2_gazebo_test_1 to _4) against a headless
# TurtleBot3 Gazebo simulation, with the machine's binary started directly so its exit status and
# shutdown are observed. Stops the machine with SIGINT at its final state and prints a report.
#
# Usage: nav2_run.sh <install-prefix> <out-dir> <1|2|3|4> [tsan|asan <overlay-prefix>]
# The install prefix holds the machine, cl_nav2z and their dependencies; nav2_bringup and the
# TurtleBot3 simulation come from the ROS installation. With tsan or asan the machine binary and its
# SMACC2 libraries are taken from the overlay prefix, built with the matching -fsanitize flags.
# Machine 4 runs in the world of machine 2, which must be in the install prefix as well.
# GUI=1 in the environment opens the Gazebo window and RViz instead of running headless.
# Exit status 0 when the final state was reached and the machine stopped without hanging.
# Up to 20 minutes. Nothing else may be running in the domain (ROS_DOMAIN_ID, default 0).
if [ $# -lt 3 ]; then echo "usage: $0 <install-prefix> <out-dir> <1|2|3|4> [tsan|asan <overlay-prefix>]"; exit 2; fi
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
INST=$(cd "$1" && pwd) || exit 2
mkdir -p "$2"; OUT=$(cd "$2" && pwd)
N=$3; MODE=${4:-plain}; OVERLAY_DIR=${5:-}; PKG=sm_nav2_gazebo_test_$N
source "$INST/setup.bash"
export RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
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
SHARE=$INST/$PKG/share/$PKG; BL=$OUT/bringup.log
SIMPROCS='[g]z sim|[c]omponent_container_isolated|[_]node --ros-args'
echo "leftover: $(pgrep -u $USER -c -f "$SIMPROCS")"
set -m
BARGS="headless:=True use_rviz:=False"; [ "${GUI:-0}" = 1 ] && BARGS="headless:=False use_rviz:=True"
if [ $N != 1 ]; then   # tests 2 and 3 bring their own world, map and parameters; test 4 uses those of test 2
  W=$SHARE; [ $N = 4 ] && W=$INST/sm_nav2_gazebo_test_2/share/sm_nav2_gazebo_test_2
  P=$OUT/nav2_params.yaml; sed "s|RUNTIMEFILL|$W/config/default_nav_to_pose_bt.xml|" $W/config/nav2_params.yaml > $P
  export GZ_SIM_RESOURCE_PATH=$W/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}
  BARGS="$BARGS params_file:=$P world:=$W/worlds/open_arena.sdf.xacro map:=$W/maps/open_arena.yaml"
fi
SMARGS="-p use_sim_time:=true"; [ $N = 2 ] && SMARGS="$SMARGS -p clear_point_distance_threshold:=0.1 -p clear_angular_distance_threshold:=3.14"
[ $N = 4 ] && SMARGS="$SMARGS -p waypoints_file:=\$(pkg_share)/config/waypoints.yaml"
( exec ros2 launch nav2_bringup tb3_simulation_launch.py $BARGS > $BL 2>&1 ) & LP=$!
for i in $(seq 1 150); do sleep 1; grep -a -q "Managed nodes are active" $BL && { echo "nav2 active after $i s"; break; }; done
grep -a -q "Managed nodes are active" $BL || echo "nav2 NOT active after 150 s"
T0=$(date +%s)
( [ -n "$OVERLAY" ] && source "$OVERLAY"; exec $PRE $BIN --ros-args -r __node:=$PKG $SMARGS --log-level INFO > $LOG 2>&1 ) & SP=$!
VERDICT=timeout; KEYSENT=0; KEYAT=0; LASTNS=0; LASTT=$(date +%s)
CPUF=$OUT/cpu.txt; : > $CPUF
( sleep 2; SMP=$(pgrep -f "^[^ ]*${PKG}_node" | head -1); while kill -0 $SP 2>/dev/null; do top -b -n 2 -d 1 -p $SMP 2>/dev/null | awk -v p=$SMP '$1==p {v=$9} END{if (v!="") print v}' >> $CPUF; sleep 1; done ) &
for i in $(seq 1 1200); do sleep 1
  grep -a -q "StFinalState\] State OnEntry code finished" $LOG && { VERDICT=final; break; }
  grep -a -q "StMissionFailed\] State OnEntry code finished" $LOG && { VERDICT=failed; break; }
  kill -0 $SP 2>/dev/null || { VERDICT=died; break; }
  NS=$(grep -a -c "State OnEntry code finished" $LOG); if [ "$NS" != "$LASTNS" ]; then LASTNS=$NS; LASTT=$(date +%s); fi
  if [ "$NS" -gt 0 ] && [ $(( $(date +%s) - LASTT )) -ge 120 ] && ! grep -a -q "StAssistedTeleopGuard\] State OnEntry code finished" $LOG; then VERDICT=stall; break; fi
  # test 3 waits in its teleop state for the n key
  if [ $N = 3 ] && grep -a -q "StAssistedTeleopGuard\] State OnEntry code finished" $LOG; then
    [ $KEYAT = 0 ] && KEYAT=$(date +%s)
    if [ $(( $(date +%s) - KEYAT )) -ge 3 ] && [ $(( ($(date +%s) - KEYAT) % 8 )) -eq 3 ]; then ros2 topic pub --once -w 1 /keyboard_unicode std_msgs/msg/UInt16 "data: 110" > /dev/null 2>&1; KEYSENT=$((KEYSENT+1)); fi
  fi
done
echo "verdict: $VERDICT after $(( $(date +%s)-T0 )) s (keys sent: $KEYSENT)"
echo "SM node CPU %: samples $(wc -l < $CPUF) mean $(awk '{s+=$1} END{if (NR) printf "%.1f", s/NR}' $CPUF) max $(sort -n $CPUF | tail -1)"
sleep 3; HANG=no
if kill -0 $SP 2>/dev/null; then T1=$(date +%s.%N); kill -INT $SP
  for i in $(seq 1 120); do kill -0 $SP 2>/dev/null || break; sleep 1; done
  if kill -0 $SP 2>/dev/null; then HANG=yes; echo "SM HANG: alive 120 s after SIGINT, SIGKILL"; kill -KILL $SP; fi
  wait $SP; echo "SM exit $? after $(python3 -c "print(round($(date +%s.%N)-$T1,1))") s"
else wait $SP; echo "SM exit $? (ended on its own)"; fi
kill -INT $LP; for i in $(seq 1 90); do kill -0 $LP 2>/dev/null || break; sleep 1; done
kill -0 $LP 2>/dev/null && { echo "bringup alive after 90 s; group SIGINT"; kill -INT -- -$LP; sleep 15; kill -0 $LP 2>/dev/null && kill -KILL -- -$LP; }
wait $LP 2>/dev/null; echo "bringup exit $?"
sleep 2; echo "leftover after: $(pgrep -u $USER -c -f "$SIMPROCS")"
echo "== states: $(grep -a -o "${PKG}::[A-Za-z0-9_]*\] State OnEntry code finished" $LOG | sed 's/.*::\(.*\)\].*/\1/' | tr '\n' ' ')"
echo "== shutdown: $(grep -a -o 'signal_handler\|Terminating scheduler\|Polling loop exited\|Shutdown complete\|Finishing State Machine\|terminate called\|destroyed with a live\|cannot be slept' $LOG | tr '\n' ' ')"
echo "== ERROR $(grep -a -c '\[ERROR\]' $LOG)  FATAL $(grep -a -c '\[FATAL\]' $LOG)  discards $(grep -a -c 'SCOPED EVENT DISCARDED' $LOG)  DDS teardown errors $(grep -a -c 'cannot publish data\|Failed to delete datareader\|Fail in delete datareader' $LOG)  SEGV $(grep -a -c 'SEGV\|Segmentation' $LOG)"
grep -a '\[ERROR\]' $LOG | sed 's/^\[[^]]*\] //' | sort | uniq -c | sort -rn | head -4 | cut -c1-160
if [ $MODE != plain ]; then
  echo "== sanitizer: ASan errors $(grep -a -c '^==[0-9]*==ERROR: AddressSanitizer' $LOG)  LSan $(grep -a -c 'LeakSanitizer' $LOG)  leak: $(grep -a -o 'SUMMARY: AddressSanitizer: [0-9]* byte(s) leaked in [0-9]* allocation' $LOG)  UBSan $(grep -a -c 'runtime error' $LOG)  TSan warnings $(cat $OUT/tsan_report.* 2>/dev/null | grep -c '^WARNING: ThreadSanitizer')"
  python3 $REPO/.github/scripts/sanitizer_reports.py $LOG $OUT/tsan_report.* 2>/dev/null | tail -3
fi
[ $VERDICT = final ] && [ $HANG = no ]
