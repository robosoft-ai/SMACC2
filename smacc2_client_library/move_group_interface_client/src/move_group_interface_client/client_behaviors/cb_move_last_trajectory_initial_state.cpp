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

#include <move_group_interface_client/client_behaviors/cb_move_last_trajectory_initial_state.hpp>
#include <move_group_interface_client/components/cp_trajectory_history.hpp>

namespace cl_move_group_interface
{
CbMoveLastTrajectoryInitialState::CbMoveLastTrajectoryInitialState() {}

CbMoveLastTrajectoryInitialState::CbMoveLastTrajectoryInitialState(int backIndex)
: backIndex_(backIndex)
{
}

CbMoveLastTrajectoryInitialState::~CbMoveLastTrajectoryInitialState() {}

void CbMoveLastTrajectoryInitialState::onEntry()
{
  CpTrajectoryHistory * trajectoryHistory;
  this->requiresComponent(trajectoryHistory);

  if (trajectoryHistory != nullptr)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;

    bool trajectoryFound = trajectoryHistory->getLastTrajectory(backIndex_, trajectory);

    if (trajectoryFound)
    {
      trajectory_msgs::msg::JointTrajectoryPoint & initialPoint =
        trajectory.joint_trajectory.points.front();

      std::stringstream ss;
      for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); i++)
      {
        auto & name = trajectory.joint_trajectory.joint_names[i];

        jointValueTarget_[name] = initialPoint.positions[i];
        ss << name << ": " << jointValueTarget_[name] << std::endl;
      }
      RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "]" << std::endl << ss.str());

      RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] move joint onEntry");
      CbMoveJoints::onEntry();
      RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] move joint onEntry finished");
    }
  }

  //call base OnEntry
}
}  // namespace cl_move_group_interface
