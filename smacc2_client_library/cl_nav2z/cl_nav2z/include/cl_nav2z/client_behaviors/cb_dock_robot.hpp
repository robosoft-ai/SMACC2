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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{

// Dock the robot via the Nav2 docking server (opennav_docking): navigate to
// the dock's staging pose, detect the dock, drive in, and confirm charging.
// Requires the docking server running with a dock database (dock-id form) or
// a detectable dock at the given pose (dock-pose form).
class CbDockRobot
: public smacc2::client_behavior_bases::CbActionClientBehaviorBase<nav2_msgs::action::DockRobot>
{
public:
  // dock at a named dock from the server's dock database
  explicit CbDockRobot(std::string dockId);

  // dock at an explicit pose; dockType selects the dock plugin when the
  // server has several configured
  CbDockRobot(geometry_msgs::msg::PoseStamped dockPose, std::string dockType = "");

  void onEntry() override;

private:
  bool useDockId_;
  std::string dockId_;
  geometry_msgs::msg::PoseStamped dockPose_;
  std::string dockType_;
};

}  // namespace cl_nav2z
