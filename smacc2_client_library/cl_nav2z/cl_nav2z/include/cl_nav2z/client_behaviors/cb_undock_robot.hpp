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

#include <string>

#include <nav2_msgs/action/undock_robot.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{

// Undock the robot via the Nav2 docking server (opennav_docking): back out of
// the dock the robot is currently on. dockType selects the dock plugin when
// the server can't infer it from the last docking action.
class CbUndockRobot
: public smacc2::client_behavior_bases::CbActionClientBehaviorBase<nav2_msgs::action::UndockRobot>
{
public:
  explicit CbUndockRobot(std::string dockType = "", float maxUndockingTime = 30.0f);

  void onEntry() override;

private:
  std::string dockType_;
  float maxUndockingTime_;
};

}  // namespace cl_nav2z
