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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/components/cp_trajectory_history.hpp>

namespace cl_move_group_interface
{
bool CpTrajectoryHistory::getLastTrajectory(
  int backIndex, moveit_msgs::msg::RobotTrajectory & trajectory)
{
  if (trajectoryHistory_.size() == 0)
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << getName() << "] requested index: " << backIndex
                       << ", history size: " << trajectoryHistory_.size());
    return false;
  }

  if (backIndex < 0)

  {
    backIndex = 0;
  }
  else if ((size_t)backIndex >= this->trajectoryHistory_.size())
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << getName() << "] requested index: " << backIndex
                       << ", history size: " << trajectoryHistory_.size());
    return false;
  }

  trajectory = this->trajectoryHistory_[this->trajectoryHistory_.size() - 1 - backIndex].trajectory;
  return true;
}

bool CpTrajectoryHistory::getLastTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory)
{
  return getLastTrajectory(-1, trajectory);
}

void CpTrajectoryHistory::pushTrajectory(
  std::string name, const moveit_msgs::msg::RobotTrajectory & trajectory,
  moveit_msgs::msg::MoveItErrorCodes result)
{
  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] adding a new trajectory to the history ( "
                     << trajectory.joint_trajectory.points.size() << " poses)");

  TrajectoryHistoryEntry entry;
  this->trajectoryHistory_.push_back(entry);

  auto & last = this->trajectoryHistory_.back();
  last.trajectory = trajectory;
  last.result = result;
  last.name = name;
}

}  // namespace cl_move_group_interface
