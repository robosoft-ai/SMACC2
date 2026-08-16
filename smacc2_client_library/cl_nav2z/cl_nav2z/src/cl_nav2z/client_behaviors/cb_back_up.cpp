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

#include <cl_nav2z/client_behaviors/cb_back_up.hpp>

namespace cl_nav2z
{

CbBackUp::CbBackUp(float distance, float speed, std::chrono::seconds timeAllowance)
: distance_(distance), speed_(speed), timeAllowance_(timeAllowance)
{
}

void CbBackUp::onEntry()
{
  Goal goal;
  goal.target.x = distance_;
  goal.speed = speed_;
  goal.time_allowance = rclcpp::Duration(timeAllowance_);

  RCLCPP_INFO(
    getLogger(), "[%s] Backing up %f m at %f m/s (collision-checked, behavior server)",
    getName().c_str(), static_cast<double>(distance_), static_cast<double>(speed_));

  sendGoal(goal);
}

}  // namespace cl_nav2z
