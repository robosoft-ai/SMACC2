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

# Transition-speed test: runs sm_coretest_transition_speed_1 with logging off and its own parameters,
# then prints the rate of its 100-iteration batches. One iteration is a round trip of two transitions.
#
# Usage: transition_speed.sh <install-prefix> [seconds] [log-file]
# Defaults: 60 seconds, ./transition_speed.log. The figure to compare between builds is the per-batch
# median; run it three times on an otherwise idle machine, since single runs vary by a few percent.
if [ $# -lt 1 ]; then echo "usage: $0 <install-prefix> [seconds] [log-file]"; exit 2; fi
INST=$(cd "$1" && pwd) || exit 2
SECS=${2:-60}; LOG=${3:-transition_speed.log}
source "$INST/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RCUTILS_COLORIZED_OUTPUT=0 RCUTILS_LOGGING_BUFFERED_STREAM=0
# A leftover ros2 daemon or node is a participant every node here would discover: stop the daemon, require an empty domain.
ros2 daemon stop > /dev/null 2>&1; trap 'ros2 daemon stop > /dev/null 2>&1' EXIT
LEFT=$(( $(ros2 node list --no-daemon -a 2>/dev/null | wc -l) - 1 ))   # the list holds this command's own node
[ "$LEFT" -le 0 ] || { echo "FAIL: $LEFT node(s) already running in ROS domain $ROS_DOMAIN_ID"; exit 2; }
PKG=$INST/sm_coretest_transition_speed_1
BIN=$PKG/lib/sm_coretest_transition_speed_1/sm_coretest_transition_speed_1_node
PARAMS=$PKG/share/sm_coretest_transition_speed_1/config/sm_coretest_transition_speed_1_config.yaml
timeout -s INT "$SECS" "$BIN" --ros-args --log-level FATAL --params-file "$PARAMS" -r __node:=sm_coretest_transition_speed_1 > "$LOG" 2>&1
echo "exit $?"
python3 -I - "$LOG" <<'PY'
import re, sys, statistics as st
hz = [float(m.group(1)) for m in re.finditer(r"seconds: ([\d.]+) Hz\. Longtime frequency: ([\d.]+) Hz", open(sys.argv[1], errors="replace").read())]
longtime = re.findall(r"Longtime frequency: ([\d.]+) Hz", open(sys.argv[1], errors="replace").read())
if not hz: print("no measurement lines"); sys.exit(1)
body = hz[5:] if len(hz) > 10 else hz   # drop warm-up batches
print(f"batches: {len(hz)} (100 transitions each)  iterations: {len(hz)*100}")
print(f"per-batch Hz: median {st.median(body):.1f}  mean {st.mean(body):.1f}  stdev {st.pstdev(body):.1f}  min {min(body):.1f}  max {max(body):.1f}")
print(f"longtime Hz (last line): {longtime[-1]}")
PY
