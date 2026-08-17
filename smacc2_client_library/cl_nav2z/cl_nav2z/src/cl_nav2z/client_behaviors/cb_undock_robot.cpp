// Copyright 2026 RobosoftAI Inc.
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

#include <cl_nav2z/client_behaviors/cb_undock_robot.hpp>

namespace cl_nav2z
{

CbUndockRobot::CbUndockRobot(std::string dockType, float maxUndockingTime)
: dockType_(std::move(dockType)), maxUndockingTime_(maxUndockingTime)
{
}

void CbUndockRobot::onEntry()
{
  Goal goal;
  goal.dock_type = dockType_;
  goal.max_undocking_time = maxUndockingTime_;

  RCLCPP_INFO(
    getLogger(), "[%s] Undocking (type '%s', max %.1f s)", getName().c_str(), dockType_.c_str(),
    static_cast<double>(maxUndockingTime_));

  sendGoal(goal);
}

}  // namespace cl_nav2z
