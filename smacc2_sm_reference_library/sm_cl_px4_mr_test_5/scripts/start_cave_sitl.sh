#!/usr/bin/env bash
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

# Terminal 1 of sm_cl_px4_mr_test_5: Gazebo (server + GUI) on the cave world,
# then PX4 SITL attached to it with the x500_lidar_3d model, then the GUI
# follow camera. Ctrl-C stops everything.
#
# Requires: `make px4_sitl` done once in $PX4_DIR (builds bin/px4, rootfs/etc and
# rootfs/gz_env.sh) and `fetch_cave_world.sh` run once.
#
#   PX4_DIR          PX4-Autopilot checkout        (default ~/workspaces/PX4-Autopilot)
#   SM5_WORLD_DIR    where fetch_cave_world.sh put the world
#   SM5_SPAWN_POSE   "x,y,z" in the gz world frame - MUST match kSpawnGz in
#                    config/mission_constants.hpp (default 2.0,0.0,0.25)
#   SM5_HEADLESS=1   no gz GUI
set -euo pipefail

# everything this terminal prints (gz server, PX4 console) also goes to a file
SITL_LOG="${SM5_SITL_LOG:-/tmp/sm_cl_px4_mr_test_5_sitl.log}"
exec > >(stdbuf -oL tee "$SITL_LOG") 2>&1
echo "== $(date) log: $SITL_LOG"

PX4_DIR="${PX4_DIR:-$HOME/workspaces/PX4-Autopilot}"
SHARE="$(ros2 pkg prefix sm_cl_px4_mr_test_5)/share/sm_cl_px4_mr_test_5"
WORLD="${SM5_WORLD_DIR:-$HOME/.gz/sm_cl_px4_mr_test_5/worlds}/cave_circuit_practice_01.sdf"
SPAWN="${SM5_SPAWN_POSE:-2.0,0.0,0.25}"
MODEL="x500_lidar_3d"
PX4_ENV="$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"

[[ -f "$WORLD" ]] || { echo "ERROR: $WORLD missing - run fetch_cave_world.sh first" >&2; exit 1; }
[[ -f "$PX4_ENV" ]] || { echo "ERROR: $PX4_ENV missing - run 'make px4_sitl' in $PX4_DIR first" >&2; exit 1; }
[[ -f "$SHARE/models/$MODEL/model.sdf" ]] || { echo "ERROR: $SHARE/models/$MODEL/model.sdf missing" >&2; exit 1; }

WORLD_NAME="$(grep -o '<world name="[^"]*"' "$WORLD" | cut -d'"' -f2)"

# PX4's gz environment: PX4 models/worlds, plugin path, server.config (world
# systems: physics, sensors ogre2, navsat, imu, ...). In standalone mode PX4
# does not source this itself, so it must be in our environment.
# shellcheck disable=SC1090
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"          # gz_env.sh appends to these
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
source "$PX4_ENV"
export GZ_SIM_RESOURCE_PATH="$SHARE/models:$GZ_SIM_RESOURCE_PATH"   # lidar_3d, x500_lidar_3d
export PX4_GZ_MODELS="$SHARE/models"                                 # PX4 spawns ${PX4_GZ_MODELS}/x500_lidar_3d/model.sdf
export GZ_IP=127.0.0.1

# only ever stop the processes this script started (another simulation may be
# running on this machine)
PIDS=()
cleanup() {
  trap - INT TERM EXIT
  echo "== stopping"
  for pid in "${PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

echo "== gz server: $WORLD (world '$WORLD_NAME')"
gz sim --verbose="${GZ_VERBOSE:-1}" -r -s "$WORLD" &
PIDS+=($!)
if [[ -z "${SM5_HEADLESS:-}" ]]; then
  gz sim -g > /dev/null 2>&1 &
  PIDS+=($!)
fi

echo "== waiting for the world (tiles load from ~/.gz/fuel; first load can take a minute)"
until gz service -i --service "/world/$WORLD_NAME/scene/info" 2>&1 | grep -q "Service providers"; do
  sleep 1
done
echo "== world ready"

echo "== PX4 SITL: model $MODEL at gz pose $SPAWN (airframe 4013 = x500 params)"
(
  cd "$PX4_DIR/build/px4_sitl_default/rootfs" &&
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD="$WORLD_NAME" \
  PX4_SYS_AUTOSTART=4013 \
  PX4_SIM_MODEL="$MODEL" \
  PX4_GZ_MODEL_POSE="$SPAWN" \
  PX4_PARAM_UXRCE_DDS_SYNCT=0 \
  PX4_PARAM_NAV_DLL_ACT=0 \
  PX4_PARAM_COM_DISARM_PRFLT=30 \
  PX4_PARAM_EKF2_GPS_DELAY=0 \
  exec ../bin/px4 -d -i 0
  # COM_DISARM_PRFLT: seconds armed without takeoff before PX4 disarms itself (default 10);
  # -d: no pxh shell (its prompt redraws flood a piped log)
) &
PIDS+=($!)

until gz model --list 2>/dev/null | grep -q "${MODEL}_0"; do
  sleep 1
done
echo "== ${MODEL}_0 spawned"

if [[ -z "${SM5_HEADLESS:-}" ]]; then
  sleep 2
  gz topic -t /gui/track -m gz.msgs.CameraTrack \
    -p "track_mode: FOLLOW follow_target { name: \"${MODEL}_0\" type: MODEL } follow_offset { x: -4 y: 0 z: 1.2 } follow_pgain: 1.0" \
    && echo "== GUI camera following ${MODEL}_0" || echo "== (follow camera request failed - set it from the GUI)"
fi

wait
