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

#pragma once

#include <nav2_msgs/action/assisted_teleop.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{

// Collision-guarded teleoperation via the Nav2 behavior server's
// AssistedTeleop action: incoming teleop velocity commands are projected
// through the local costmap and scaled/zeroed before a collision. The action
// runs for the whole time allowance (success on expiry); leaving the state
// early cancels it through the behavior base.
class CbAssistedTeleop : public smacc2::client_behavior_bases::CbActionClientBehaviorBase<
                           nav2_msgs::action::AssistedTeleop>
{
public:
  explicit CbAssistedTeleop(std::chrono::seconds timeAllowance = std::chrono::seconds(30));

  void onEntry() override;

private:
  std::chrono::seconds timeAllowance_;
};

}  // namespace cl_nav2z
