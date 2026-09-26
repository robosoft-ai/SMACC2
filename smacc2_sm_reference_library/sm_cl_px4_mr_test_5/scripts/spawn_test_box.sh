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

# Negative test helper: drop a static 2 m box into the running cave world so
# it enters the lidar's forward safety cone.
#   spawn_test_box.sh X Y Z   (gz world frame; e.g. 45 0 3 on the entrance corridor)
set -euo pipefail
X="${1:?x}"; Y="${2:?y}"; Z="${3:?z}"
WORLD="${SM5_WORLD_NAME:-cave_circuit_practice_01}"
SDF="<sdf version=\\\"1.9\\\"><model name=\\\"test_box\\\"><static>true</static><pose>$X $Y $Z 0 0 0</pose><link name=\\\"l\\\"><visual name=\\\"v\\\"><geometry><box><size>2 2 2</size></box></geometry><material><diffuse>1 0.2 0.2 1</diffuse></material></visual><collision name=\\\"c\\\"><geometry><box><size>2 2 2</size></box></geometry></collision></link></model></sdf>"
gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req "sdf: \"$SDF\", name: \"test_box\", allow_renaming: false"
echo "test_box spawned at $X $Y $Z in world $WORLD"
