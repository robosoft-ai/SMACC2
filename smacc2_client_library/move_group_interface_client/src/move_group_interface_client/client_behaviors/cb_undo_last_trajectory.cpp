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

#include <move_group_interface_client/client_behaviors/cb_undo_last_trajectory.hpp>
#include <move_group_interface_client/components/cp_trajectory_history.hpp>

namespace cl_move_group_interface
{
CbUndoLastTrajectory::CbUndoLastTrajectory() {}

CbUndoLastTrajectory::CbUndoLastTrajectory(int backIndex) : backIndex_(backIndex) {}

CbUndoLastTrajectory::~CbUndoLastTrajectory() {}

void CbUndoLastTrajectory::generateTrajectory() {}

void CbUndoLastTrajectory::onEntry()
{
  CpTrajectoryHistory * trajectoryHistory;
  this->requiresComponent(trajectoryHistory);
  this->requiresClient(movegroupClient_);

  if (trajectoryHistory->getLastTrajectory(backIndex_, trajectory))
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << getName() << "] reversing last trajectory [" << backIndex_ << "]");

    auto initialTime = trajectory.joint_trajectory.points.back().time_from_start;

    reversed = trajectory;

    std::reverse(reversed.joint_trajectory.points.begin(), reversed.joint_trajectory.points.end());

    for (auto & jp : reversed.joint_trajectory.points)
    {
      jp.time_from_start = rclcpp::Duration(initialTime) - rclcpp::Duration(jp.time_from_start);
    }

    this->executeJointSpaceTrajectory(reversed);
  }
  else
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << getName() << "] could not undo last trajectory, trajectory not found.");
  }
}

}  // namespace cl_move_group_interface
