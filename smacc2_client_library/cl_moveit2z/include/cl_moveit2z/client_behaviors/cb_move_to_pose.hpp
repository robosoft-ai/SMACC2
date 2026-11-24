// Copyright 2026 Robosoft Inc.
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
 * 	 Authors: Brett Aldrich
 *
 *****************************************************************************************************************/

#pragma once

#include <optional>
#include <string>

#include <cl_moveit2z/client_behaviors/cb_moveit2z_client_behavior_base.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace cl_moveit2z
{

struct MoveToPoseParams
{
  geometry_msgs::msg::PoseStamped pose;  // of the tip link, in the frame of its header
  std::string tipLink;                   // empty: the end effector link of the client's group
  double velocityScaling = 1.0;          // fraction of the joint velocity limits
  double accelerationScaling = 1.0;      // fraction of the joint acceleration limits
  double planningTime = 1.0;             // s
};

// Moves the tip link of its client's planning group to a pose: one plan, then its execution.
// Without parameters it does nothing and posts no event, so one can stand idle on an orthogonal.
class CbMoveToPose : public CbMoveit2zClientBehaviorBase
{
public:
  CbMoveToPose() = default;

  explicit CbMoveToPose(const MoveToPoseParams & params) : params_(params) {}

  void setParams(const MoveToPoseParams & params) { params_ = params; }

  void onEntry() override
  {
    if (!params_)
    {
      RCLCPP_INFO(getLogger(), "[CbMoveToPose] no pose given: idle");
      return;
    }
    if (cpMotionPlanner_ == nullptr)
    {
      RCLCPP_ERROR(getLogger(), "[CbMoveToPose] the client has no CpMotionPlanner");
      this->postMotionFailure();
      return;
    }

    PlanningOptions options;
    options.planningTime = params_->planningTime;
    options.maxVelocityScaling = params_->velocityScaling;
    options.maxAccelerationScaling = params_->accelerationScaling;
    const std::optional<std::string> tipLink =
      params_->tipLink.empty() ? std::nullopt : std::optional<std::string>(params_->tipLink);

    const auto planned = cpMotionPlanner_->planToPose(params_->pose, tipLink, options);
    if (!planned.success)
    {
      RCLCPP_WARN(getLogger(), "[CbMoveToPose] no plan: %s", planned.errorMessage.c_str());
      this->postMotionFailure();
      return;
    }

    bool executed = false;
    if (cpTrajectoryExecutor_ != nullptr)
    {
      ExecutionOptions executionOptions;
      executionOptions.trajectoryName = this->getName();
      executed = cpTrajectoryExecutor_->executePlan(planned.plan, executionOptions).success;
    }
    else
    {
      executed = cpMoveGroup_->moveGroupClientInterface->execute(planned.plan) ==
                 moveit::core::MoveItErrorCode::SUCCESS;
    }

    if (executed)
    {
      this->postMotionSuccess();
    }
    else
    {
      RCLCPP_WARN(getLogger(), "[CbMoveToPose] the execution failed");
      this->postMotionFailure();
    }
  }

private:
  std::optional<MoveToPoseParams> params_;
};

}  // namespace cl_moveit2z
