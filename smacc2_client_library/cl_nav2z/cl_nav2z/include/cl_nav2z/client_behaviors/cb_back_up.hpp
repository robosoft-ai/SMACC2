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

#include <nav2_msgs/action/back_up.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{

// Collision-checked straight backward translation via the Nav2 behavior
// server's BackUp action: odometry-relative, no planner/controller stack
// involved (compare CbNavigateBackwards, which runs the full Nav2 pipeline).
class CbBackUp
: public smacc2::client_behavior_bases::CbActionClientBehaviorBase<nav2_msgs::action::BackUp>
{
public:
  CbBackUp(
    float distance, float speed = 0.15f,
    std::chrono::seconds timeAllowance = std::chrono::seconds(30));

  void onEntry() override;

private:
  float distance_;
  float speed_;
  std::chrono::seconds timeAllowance_;
};

}  // namespace cl_nav2z
