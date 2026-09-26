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

# One-time: download the DARPA SubT "Cave Circuit Practice 01" world (Open
# Robotics, CC-BY-4.0) and its tiles from Gazebo Fuel, patch it for PX4 SITL
# and place it where start_cave_sitl.sh expects it.
#
#   fetch_cave_world.sh            # ~100 MB of tiles into ~/.gz/fuel
#   SM5_WORLD_DIR=... fetch_cave_world.sh
set -euo pipefail

WORLD_URL="https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Cave Circuit Practice 01"
WORLD_FILE="cave_circuit_practice_01.sdf"
OUT_DIR="${SM5_WORLD_DIR:-$HOME/.gz/sm_cl_px4_mr_test_5/worlds}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== downloading world: $WORLD_URL"
gz fuel download -u "$WORLD_URL"

SRC="$(find "$HOME/.gz/fuel" -name "$WORLD_FILE" | head -n 1)"
if [[ -z "$SRC" ]]; then
  echo "ERROR: $WORLD_FILE not found under ~/.gz/fuel after download" >&2
  exit 1
fi
echo "== source world: $SRC"

mkdir -p "$OUT_DIR"
python3 "$SCRIPT_DIR/patch_cave_world.py" "$SRC" "$OUT_DIR/$WORLD_FILE"

echo "== prefetching tile models (skips ones already cached)"
grep -o '<uri>[^<]*</uri>' "$OUT_DIR/$WORLD_FILE" \
  | sed 's/<uri>//; s/<\/uri>//' | sort -u \
  | while read -r uri; do
      name="${uri##*/}"
      if compgen -G "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/${name,,}/*/model.sdf" > /dev/null; then
        continue
      fi
      echo "   $name"
      gz fuel download -u "$uri" > /dev/null
    done

echo "== done: $OUT_DIR/$WORLD_FILE"
echo "   world name: $(grep -o '<world name="[^"]*"' "$OUT_DIR/$WORLD_FILE" | cut -d'"' -f2)"
