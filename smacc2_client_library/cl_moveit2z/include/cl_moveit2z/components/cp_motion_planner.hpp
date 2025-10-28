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

#include <smacc2/component.hpp>

#include <cl_moveit2z/cl_moveit2z.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <map>
#include <optional>
#include <string>

namespace cl_moveit2z
{
/**
 * @brief Configuration options for motion planning
 */
struct PlanningOptions
{
  std::optional<double> planningTime;
  std::optional<double> maxVelocityScaling;
  std::optional<double> maxAccelerationScaling;
  std::optional<std::string> planningPipelineId;
  std::optional<std::string> plannerId;
  std::optional<std::string> poseReferenceFrame;
};

/**
 * @brief Result of a planning operation
 */
struct PlanningResult
{
  bool success;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode errorCode;
  std::string errorMessage;

  PlanningResult()
  : success(false), errorCode(moveit::core::MoveItErrorCode::FAILURE), errorMessage("")
  {
  }
};

/**
 * @brief Component for centralized motion planning operations
 *
 * This component wraps all MoveGroupInterface planning operations to provide
 * a consistent interface for planning motions. It supports planning to:
 * - Cartesian poses
 * - Joint configurations
 * - Named targets
 * - Cartesian paths
 *
 * Pattern: API wrapper component with result types
 */
class CpMotionPlanner : public smacc2::ISmaccComponent
{
public:
  CpMotionPlanner() = default;
  virtual ~CpMotionPlanner() = default;

  /**
   * @brief Initialize the component and get reference to ClMoveit2z client
   */
  inline void onInitialize() override
  {
    this->requiresClient(moveit2zClient_);
    RCLCPP_INFO(getLogger(), "[CpMotionPlanner] Component initialized");
  }

  /**
   * @brief Plan to a Cartesian pose
   *
   * @param target Target pose (position and orientation)
   * @param tipLink End effector tip link (optional, uses default if not specified)
   * @param options Planning configuration options
   * @return PlanningResult with success status and computed plan
   */
  inline PlanningResult planToPose(
    const geometry_msgs::msg::PoseStamped & target,
    const std::optional<std::string> & tipLink = std::nullopt, const PlanningOptions & options = {})
  {
    PlanningResult result;

    if (!moveit2zClient_)
    {
      result.errorMessage = "ClMoveit2z client not available";
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

    try
    {
      // Apply planning options
      applyPlanningOptions(*moveGroup, options);

      // Set pose target
      if (tipLink && !tipLink->empty())
      {
        moveGroup->setPoseTarget(target, *tipLink);
      }
      else
      {
        moveGroup->setPoseTarget(target);
      }

      // Set reference frame if specified
      if (options.poseReferenceFrame)
      {
        moveGroup->setPoseReferenceFrame(*options.poseReferenceFrame);
      }
      else if (!target.header.frame_id.empty())
      {
        moveGroup->setPoseReferenceFrame(target.header.frame_id);
      }

      RCLCPP_INFO(getLogger(), "[CpMotionPlanner] Planning to pose...");

      // Plan
      result.errorCode = moveGroup->plan(result.plan);
      result.success = (result.errorCode == moveit::core::MoveItErrorCode::SUCCESS);

      if (result.success)
      {
        RCLCPP_INFO(getLogger(), "[CpMotionPlanner] Planning to pose succeeded");
      }
      else
      {
        result.errorMessage =
          "Planning to pose failed with error code: " + std::to_string(result.errorCode.val);
        RCLCPP_WARN(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      }
    }
    catch (const std::exception & e)
    {
      result.success = false;
      result.errorMessage = std::string("Exception during planning: ") + e.what();
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
    }

    return result;
  }

  /**
   * @brief Plan to joint values
   *
   * @param jointTargets Map of joint names to target values
   * @param options Planning configuration options
   * @return PlanningResult with success status and computed plan
   */
  inline PlanningResult planToJointTarget(
    const std::map<std::string, double> & jointTargets, const PlanningOptions & options = {})
  {
    PlanningResult result;

    if (!moveit2zClient_)
    {
      result.errorMessage = "ClMoveit2z client not available";
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    if (jointTargets.empty())
    {
      result.errorMessage = "No joint targets specified";
      RCLCPP_WARN(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

    try
    {
      // Apply planning options
      applyPlanningOptions(*moveGroup, options);

      // Set joint value targets
      moveGroup->setJointValueTarget(jointTargets);

      RCLCPP_INFO(
        getLogger(), "[CpMotionPlanner] Planning to joint configuration (%lu joints)...",
        jointTargets.size());

      // Plan
      result.errorCode = moveGroup->plan(result.plan);
      result.success = (result.errorCode == moveit::core::MoveItErrorCode::SUCCESS);

      if (result.success)
      {
        RCLCPP_INFO(getLogger(), "[CpMotionPlanner] Planning to joint configuration succeeded");
      }
      else
      {
        result.errorMessage = "Planning to joint configuration failed with error code: " +
                              std::to_string(result.errorCode.val);
        RCLCPP_WARN(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      }
    }
    catch (const std::exception & e)
    {
      result.success = false;
      result.errorMessage = std::string("Exception during planning: ") + e.what();
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
    }

    return result;
  }

  /**
   * @brief Plan a Cartesian path
   *
   * @param waypoints Vector of waypoint poses
   * @param maxStep Maximum distance between path points (meters)
   * @param jumpThreshold Maximum joint jump threshold (0.0 = disabled)
   * @param avoidCollisions Whether to check for collisions
   * @return PlanningResult with success status and computed trajectory
   */
  inline PlanningResult planCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints, double maxStep = 0.01,
    double jumpThreshold = 0.0, bool avoidCollisions = true)
  {
    PlanningResult result;

    if (!moveit2zClient_)
    {
      result.errorMessage = "ClMoveit2z client not available";
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    if (waypoints.empty())
    {
      result.errorMessage = "No waypoints specified";
      RCLCPP_WARN(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

    try
    {
      RCLCPP_INFO(
        getLogger(), "[CpMotionPlanner] Planning Cartesian path with %lu waypoints...",
        waypoints.size());

      moveit_msgs::msg::RobotTrajectory trajectory;
      double fractionAchieved = moveGroup->computeCartesianPath(
        waypoints, maxStep, jumpThreshold, trajectory, avoidCollisions);

      result.plan.trajectory_ = trajectory;

      // Consider successful if we achieved at least 95% of the path
      result.success = (fractionAchieved >= 0.95);

      if (result.success)
      {
        result.errorCode = moveit::core::MoveItErrorCode::SUCCESS;
        RCLCPP_INFO(
          getLogger(), "[CpMotionPlanner] Cartesian path planning succeeded (%.1f%% achieved)",
          fractionAchieved * 100.0);
      }
      else
      {
        result.errorCode = moveit::core::MoveItErrorCode::PLANNING_FAILED;
        result.errorMessage = "Cartesian path planning achieved only " +
                              std::to_string(fractionAchieved * 100.0) + "% of the path";
        RCLCPP_WARN(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
      }
    }
    catch (const std::exception & e)
    {
      result.success = false;
      result.errorMessage = std::string("Exception during Cartesian path planning: ") + e.what();
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] %s", result.errorMessage.c_str());
    }

    return result;
  }

  /**
   * @brief Get current robot state
   *
   * @param waitTime Maximum time to wait for state (seconds)
   * @return Pointer to current robot state, or nullptr if unavailable
   */
  inline moveit::core::RobotStatePtr getCurrentState(double waitTime = 1.0)
  {
    if (!moveit2zClient_)
    {
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] ClMoveit2z client not available");
      return nullptr;
    }

    auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

    try
    {
      return moveGroup->getCurrentState(waitTime);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(getLogger(), "[CpMotionPlanner] Exception getting current state: %s", e.what());
      return nullptr;
    }
  }

private:
  ClMoveit2z * moveit2zClient_ = nullptr;

  /**
   * @brief Apply planning options to MoveGroupInterface
   *
   * @param moveGroup Reference to MoveGroupInterface
   * @param options Planning options to apply
   */
  inline void applyPlanningOptions(
    moveit::planning_interface::MoveGroupInterface & moveGroup, const PlanningOptions & options)
  {
    if (options.planningTime)
    {
      moveGroup.setPlanningTime(*options.planningTime);
    }

    if (options.maxVelocityScaling)
    {
      moveGroup.setMaxVelocityScalingFactor(*options.maxVelocityScaling);
    }

    if (options.maxAccelerationScaling)
    {
      moveGroup.setMaxAccelerationScalingFactor(*options.maxAccelerationScaling);
    }

    if (options.planningPipelineId)
    {
      moveGroup.setPlanningPipelineId(*options.planningPipelineId);
    }

    if (options.plannerId)
    {
      moveGroup.setPlannerId(*options.plannerId);
    }
  }
};

}  // namespace cl_moveit2z
