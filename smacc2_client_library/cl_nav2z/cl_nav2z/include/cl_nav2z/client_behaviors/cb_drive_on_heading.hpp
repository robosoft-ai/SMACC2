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

#include <nav2_msgs/action/drive_on_heading.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{

// Collision-checked straight forward translation along the current heading via
// the Nav2 behavior server's DriveOnHeading action: odometry-relative dead
// reckoning - works even when localization is degraded (compare
// CbNavigateForward, which needs the full planner/controller/localization stack).
class CbDriveOnHeading : public smacc2::client_behavior_bases::CbActionClientBehaviorBase<
                           nav2_msgs::action::DriveOnHeading>
{
public:
  CbDriveOnHeading(
    float distance, float speed = 0.15f,
    std::chrono::seconds timeAllowance = std::chrono::seconds(30));

  void onEntry() override;

private:
  float distance_;
  float speed_;
  std::chrono::seconds timeAllowance_;
};

}  // namespace cl_nav2z
