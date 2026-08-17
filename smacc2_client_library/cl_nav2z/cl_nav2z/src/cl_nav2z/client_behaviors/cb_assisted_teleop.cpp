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

#include <cl_nav2z/client_behaviors/cb_assisted_teleop.hpp>

namespace cl_nav2z
{

CbAssistedTeleop::CbAssistedTeleop(std::chrono::seconds timeAllowance)
: timeAllowance_(timeAllowance)
{
}

void CbAssistedTeleop::onEntry()
{
  Goal goal;
  goal.time_allowance = rclcpp::Duration(timeAllowance_);

  if (timeAllowance_.count() > 0)
  {
    RCLCPP_INFO(
      getLogger(), "[%s] Assisted teleop for %ld s (collision-guarded, behavior server)",
      getName().c_str(), static_cast<long>(timeAllowance_.count()));
  }
  else
  {
    RCLCPP_INFO(
      getLogger(),
      "[%s] Assisted teleop, unlimited window (collision-guarded; ends on cancellation)",
      getName().c_str());
  }

  sendGoal(goal);
}

void CbAssistedTeleop::onActionAbort(const WrappedResult & result)
{
  if (
    result.result != nullptr &&
    result.result->error_code == nav2_msgs::action::AssistedTeleop::Result::TIMEOUT)
  {
    RCLCPP_INFO(
      getLogger(), "[%s] Teleop window completed (time allowance expired) - success",
      getName().c_str());
    this->postSuccessEvent();
    return;
  }

  smacc2::client_behavior_bases::CbActionClientBehaviorBase<
    nav2_msgs::action::AssistedTeleop>::onActionAbort(result);
}

}  // namespace cl_nav2z
