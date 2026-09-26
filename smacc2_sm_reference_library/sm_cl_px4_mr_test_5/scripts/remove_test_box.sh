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

# Remove the box placed by spawn_test_box.sh.
set -euo pipefail
WORLD="${SM5_WORLD_NAME:-cave_circuit_practice_01}"
gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "test_box", type: MODEL'
echo "test_box removed from world $WORLD"
