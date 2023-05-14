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

#include <future>
<<<<<<< HEAD:smacc2_client_library/moveit2z_client/src/moveit2z_client/client_behaviors/cb_move_joints.cpp
#include <moveit2z_client/client_behaviors/cb_move_joints.hpp>
=======
#include <moveit2z_client/client_behaviors/cb_move_joints.hpp>
>>>>>>> 056c654b26293282493ab9a4aaec5399f25f061f:smacc2_client_library/moveit2z_client/src/moveit2z_client/client_behaviors/cb_move_joints.cpp

namespace cl_moveit2z
{
  CbMoveJoints::CbMoveJoints(const std::map<std::string, double> & jointValueTarget)
  : jointValueTarget_(jointValueTarget)
  {
  }

  CbMoveJoints::CbMoveJoints() {}

  void CbMoveJoints::onEntry()
  {
    this->requiresClient(movegroupClient_);

    if (this->group_)
    {
      moveit::planning_interface::MoveGroupInterface move_group(
        getNode(), moveit::planning_interface::MoveGroupInterface::Options(*(this->group_)));
      this->moveJoints(move_group);
    }
    else
    {
      this->moveJoints(*movegroupClient_->moveGroupClientInterface);
    }
  }

  std::string currentJointStatesToString(
    moveit::planning_interface::MoveGroupInterface & moveGroupInterface,
    std::map<std::string, double> & targetJoints)
  {
    auto state = moveGroupInterface.getCurrentState();

    if (state == nullptr) return std::string();

    auto vnames = state->getVariableNames();

    std::stringstream ss;

    for (auto & tgj : targetJoints)
    {
      auto it = std::find(vnames.begin(), vnames.end(), tgj.first);
      auto index = std::distance(vnames.begin(), it);

      ss << tgj.first << ":" << state->getVariablePosition(index) << std::endl;
    }

    return ss.str();
  }

  void CbMoveJoints::moveJoints(moveit::planning_interface::MoveGroupInterface & moveGroupInterface)
  {
    if (scalingFactor_) moveGroupInterface.setMaxVelocityScalingFactor(*scalingFactor_);

    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

    if (jointValueTarget_.size() == 0)
    {
      RCLCPP_WARN(
        getLogger(), "[CbMoveJoints] No joint was value specified. Skipping planning call.");
      success = false;
    }
    else
    {
      moveGroupInterface.setJointValueTarget(jointValueTarget_);
      //moveGroupInterface.setGoalJointTolerance(0.01);

      auto result = moveGroupInterface.plan(computedMotionPlan);

      success = (result == moveit::core::MoveItErrorCode::SUCCESS);

      RCLCPP_INFO(
        getLogger(), "[CbMoveJoints] Execution plan result %s (%d)", success ? "SUCCESS" : "FAILED",
        result.val);
    }

    if (success)
    {
      auto executionResult = moveGroupInterface.execute(computedMotionPlan);

      //auto statestr = currentJointStatesToString(moveGroupInterface, jointValueTarget_);

      if (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << this->getName()
                           << "] motion execution succeeded. Throwing success event. " << std::endl
          //                         << statestr
        );
        movegroupClient_->postEventMotionExecutionSucceded();
        this->postSuccessEvent();
      }
      else
      {
        RCLCPP_WARN_STREAM(
          getLogger(),
          "[" << this->getName() << "] motion execution failed. Throwing fail event." << std::endl
          //                         << statestr
        );
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
    }
    else
    {
      auto statestr = currentJointStatesToString(moveGroupInterface, jointValueTarget_);
      RCLCPP_WARN_STREAM(
        getLogger(),
        "[" << this->getName() << "] motion execution failed. Throwing fail event." << std::endl
        //                       << statestr
      );
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }

  void CbMoveJoints::onExit() {}
}  // namespace cl_moveit2z
