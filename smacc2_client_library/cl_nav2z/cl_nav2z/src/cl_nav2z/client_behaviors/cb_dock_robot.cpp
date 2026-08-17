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

#include <cl_nav2z/client_behaviors/cb_dock_robot.hpp>

namespace cl_nav2z
{

CbDockRobot::CbDockRobot(std::string dockId) : useDockId_(true), dockId_(std::move(dockId)) {}

CbDockRobot::CbDockRobot(geometry_msgs::msg::PoseStamped dockPose, std::string dockType)
: useDockId_(false), dockPose_(std::move(dockPose)), dockType_(std::move(dockType))
{
}

void CbDockRobot::onEntry()
{
  Goal goal;
  goal.use_dock_id = useDockId_;
  if (useDockId_)
  {
    goal.dock_id = dockId_;
    RCLCPP_INFO(getLogger(), "[%s] Docking at dock '%s'", getName().c_str(), dockId_.c_str());
  }
  else
  {
    goal.dock_pose = dockPose_;
    goal.dock_type = dockType_;
    RCLCPP_INFO(
      getLogger(), "[%s] Docking at pose [%.2f, %.2f] (type '%s')", getName().c_str(),
      dockPose_.pose.position.x, dockPose_.pose.position.y, dockType_.c_str());
  }

  sendGoal(goal);
}

}  // namespace cl_nav2z
