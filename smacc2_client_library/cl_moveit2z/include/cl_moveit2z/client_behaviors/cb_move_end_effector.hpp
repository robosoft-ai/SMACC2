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

#include <tf2/impl/utils.h>
#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/common.hpp>
#include <cl_moveit2z/components/cp_motion_planner.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <future>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace cl_moveit2z
{
class CbMoveEndEffector : public smacc2::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::msg::PoseStamped targetPose;
  std::string tip_link_;
  std::optional<std::string> group_;

  CbMoveEndEffector() {}

  CbMoveEndEffector(geometry_msgs::msg::PoseStamped target_pose, std::string tip_link = "")
  : targetPose(target_pose)
  {
    tip_link_ = tip_link;
  }

  virtual void onEntry() override
  {
    this->requiresClient(movegroupClient_);

    if (this->group_)
    {
      RCLCPP_DEBUG(
        getLogger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
      moveit::planning_interface::MoveGroupInterface move_group(getNode(), *group_);
      this->moveToAbsolutePose(move_group, targetPose);
      RCLCPP_DEBUG(
        getLogger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
    }
    else
    {
      RCLCPP_DEBUG(
        getLogger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
      this->moveToAbsolutePose(*(movegroupClient_->moveGroupClientInterface), targetPose);
      RCLCPP_DEBUG(
        getLogger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
    }
  }

protected:
  bool moveToAbsolutePose(
    moveit::planning_interface::MoveGroupInterface & moveGroupInterface,
    geometry_msgs::msg::PoseStamped & targetObjectPose)
  {
    RCLCPP_DEBUG(getLogger(), "[CbMoveEndEffector] Synchronous sleep of 1 seconds");
    rclcpp::sleep_for(500ms);

    // Try to use CpMotionPlanner component (preferred)
    CpMotionPlanner * motionPlanner = nullptr;
    this->requiresComponent(motionPlanner, false);  // Optional component

    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

    RCLCPP_INFO_STREAM(
      getLogger(), "[CbMoveEndEffector] Target End effector Pose: " << targetObjectPose);

    if (motionPlanner != nullptr)
    {
      // Use component-based motion planner (preferred)
      RCLCPP_INFO(getLogger(), "[CbMoveEndEffector] Using CpMotionPlanner component for planning");

      PlanningOptions options;
      options.planningTime = 1.0;
      options.poseReferenceFrame = targetObjectPose.header.frame_id;

      std::optional<std::string> tipLinkOpt;
      if (!tip_link_.empty())
      {
        tipLinkOpt = tip_link_;
      }

      auto result = motionPlanner->planToPose(targetObjectPose, tipLinkOpt, options);

      success = result.success;
      if (success)
      {
        computedMotionPlan = result.plan;
        RCLCPP_INFO(getLogger(), "[CbMoveEndEffector] Planning succeeded (via CpMotionPlanner)");
      }
      else
      {
        RCLCPP_WARN(
          getLogger(), "[CbMoveEndEffector] Planning failed (via CpMotionPlanner): %s",
          result.errorMessage.c_str());
      }
    }
    else
    {
      // Fallback to legacy direct API calls
      RCLCPP_WARN(
        getLogger(),
        "[CbMoveEndEffector] CpMotionPlanner component not available, using legacy planning "
        "(consider adding CpMotionPlanner component)");

      moveGroupInterface.setPlanningTime(1.0);
      moveGroupInterface.setPoseTarget(targetObjectPose, tip_link_);
      moveGroupInterface.setPoseReferenceFrame(targetObjectPose.header.frame_id);

      success =
        (moveGroupInterface.plan(computedMotionPlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        getLogger(), "[CbMoveEndEffector] Planning %s (legacy mode)",
        success ? "succeeded" : "FAILED");
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
          getLogger(), "[CbMoveEndEffector] Using CpTrajectoryExecutor component for execution");

        ExecutionOptions execOptions;
        execOptions.trajectoryName = this->getName();

        auto execResult = trajectoryExecutor->executePlan(computedMotionPlan, execOptions);
        executionSuccess = execResult.success;

        if (executionSuccess)
        {
          RCLCPP_INFO(
            getLogger(), "[CbMoveEndEffector] Execution succeeded (via CpTrajectoryExecutor)");
        }
        else
        {
          RCLCPP_WARN(
            getLogger(), "[CbMoveEndEffector] Execution failed (via CpTrajectoryExecutor): %s",
            execResult.errorMessage.c_str());
        }
      }
      else
      {
        // Fallback to legacy direct execution
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveEndEffector] CpTrajectoryExecutor component not available, using legacy "
          "execution "
          "(consider adding CpTrajectoryExecutor component)");

        auto executionResult = moveGroupInterface.execute(computedMotionPlan);
        executionSuccess = (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

        RCLCPP_INFO(
          getLogger(), "[CbMoveEndEffector] Execution %s (legacy mode)",
          executionSuccess ? "succeeded" : "failed");
      }

      // Post events
      if (executionSuccess)
      {
        movegroupClient_->postEventMotionExecutionSucceded();
        this->postSuccessEvent();
      }
      else
      {
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
    }
    else
    {
      RCLCPP_INFO(getLogger(), "[CbMoveEndEffector] planning failed, skipping execution");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }

    RCLCPP_DEBUG(getLogger(), "[CbMoveEndEffector] Synchronous sleep of 1 seconds");
    rclcpp::sleep_for(500ms);

    return success;
  }

  ClMoveit2z * movegroupClient_;
};
}  // namespace cl_moveit2z
