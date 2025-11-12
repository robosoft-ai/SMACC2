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
 *****************************************************************************************************************/

#pragma once

#include <algorithm>
#include <future>
#include <map>
#include <sstream>
#include <string>

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/components/cp_motion_planner.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_moveit2z
{
class CbMoveJoints : public smacc2::SmaccAsyncClientBehavior
{
public:
  std::optional<double> scalingFactor_;
  std::map<std::string, double> jointValueTarget_;
  std::optional<std::string> group_;

  CbMoveJoints() {}

  CbMoveJoints(const std::map<std::string, double> & jointValueTarget)
  : jointValueTarget_(jointValueTarget)
  {
  }

  virtual void onEntry() override
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

  virtual void onExit() override {}

protected:
  static std::string currentJointStatesToString(
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

  void moveJoints(moveit::planning_interface::MoveGroupInterface & moveGroupInterface)
  {
    if (jointValueTarget_.size() == 0)
    {
      RCLCPP_WARN(getLogger(), "[CbMoveJoints] No joint value specified. Skipping planning call.");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
      return;
    }

    // Try to use CpMotionPlanner component (preferred)
    CpMotionPlanner * motionPlanner = nullptr;
    this->requiresComponent(motionPlanner, false);  // Optional component

    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

    if (motionPlanner != nullptr)
    {
      // Use component-based motion planner (preferred)
      RCLCPP_INFO(getLogger(), "[CbMoveJoints] Using CpMotionPlanner component for joint planning");

      PlanningOptions options;
      if (scalingFactor_)
      {
        options.maxVelocityScaling = *scalingFactor_;
      }

      auto result = motionPlanner->planToJointTarget(jointValueTarget_, options);

      success = result.success;
      if (success)
      {
        computedMotionPlan = result.plan;
        RCLCPP_INFO(getLogger(), "[CbMoveJoints] Planning succeeded (via CpMotionPlanner)");
      }
      else
      {
        RCLCPP_WARN(
          getLogger(), "[CbMoveJoints] Planning failed (via CpMotionPlanner): %s",
          result.errorMessage.c_str());
      }
    }
    else
    {
      // Fallback to legacy direct API calls
      RCLCPP_WARN(
        getLogger(),
        "[CbMoveJoints] CpMotionPlanner component not available, using legacy planning "
        "(consider adding CpMotionPlanner component)");

      if (scalingFactor_) moveGroupInterface.setMaxVelocityScalingFactor(*scalingFactor_);

      moveGroupInterface.setJointValueTarget(jointValueTarget_);

      auto result = moveGroupInterface.plan(computedMotionPlan);

      success = (result == moveit::core::MoveItErrorCode::SUCCESS);

      RCLCPP_INFO(
        getLogger(), "[CbMoveJoints] Planning %s (legacy mode, code: %d)",
        success ? "SUCCESS" : "FAILED", result.val);
    }

    // Execution
    if (success)
    {
      // Try to use CpTrajectoryExecutor component (preferred)
      CpTrajectoryExecutor * trajectoryExecutor = nullptr;
      this->requiresComponent(trajectoryExecutor, false);  // Optional component

      bool executionSuccess = false;

      if (trajectoryExecutor != nullptr)
      {
        // Use component-based trajectory executor (preferred)
        RCLCPP_INFO(
          getLogger(), "[CbMoveJoints] Using CpTrajectoryExecutor component for execution");

        ExecutionOptions execOptions;
        execOptions.trajectoryName = this->getName();
        if (scalingFactor_)
        {
          execOptions.maxVelocityScaling = *scalingFactor_;
        }

        auto execResult = trajectoryExecutor->executePlan(computedMotionPlan, execOptions);
        executionSuccess = execResult.success;

        if (executionSuccess)
        {
          RCLCPP_INFO(getLogger(), "[CbMoveJoints] Execution succeeded (via CpTrajectoryExecutor)");
        }
        else
        {
          RCLCPP_WARN(
            getLogger(), "[CbMoveJoints] Execution failed (via CpTrajectoryExecutor): %s",
            execResult.errorMessage.c_str());
        }
      }
      else
      {
        // Fallback to legacy direct execution
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveJoints] CpTrajectoryExecutor component not available, using legacy execution "
          "(consider adding CpTrajectoryExecutor component)");

        auto executionResult = moveGroupInterface.execute(computedMotionPlan);
        executionSuccess = (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

        RCLCPP_INFO(
          getLogger(), "[CbMoveJoints] Execution %s (legacy mode)",
          executionSuccess ? "succeeded" : "failed");
      }

      // Post events
      if (executionSuccess)
      {
        RCLCPP_INFO_STREAM(
          getLogger(),
          "[" << this->getName() << "] motion execution succeeded. Throwing success event.");
        movegroupClient_->postEventMotionExecutionSucceded();
        this->postSuccessEvent();
      }
      else
      {
        RCLCPP_WARN_STREAM(
          getLogger(), "[" << this->getName() << "] motion execution failed. Throwing fail event.");
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
    }
    else
    {
      auto statestr = currentJointStatesToString(moveGroupInterface, jointValueTarget_);
      RCLCPP_WARN_STREAM(
        getLogger(), "[" << this->getName() << "] planning failed. Throwing fail event."
                         << std::endl
                         << statestr);
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }

  ClMoveit2z * movegroupClient_;
};
}  // namespace cl_moveit2z
