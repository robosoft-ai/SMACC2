// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_event_dispatcher.hpp>

namespace cl_move_base_z
{
void WaypointEventDispatcher::postWaypointEvent(int index)
{
  auto & fn = postWaypointFn[index % WAYPOINTS_EVENTCOUNT];
  if (fn != nullptr) fn();
}
}  // namespace cl_move_base_z
