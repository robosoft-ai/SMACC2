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
#include <cl_moveit2z/components/cp_trajectory_history.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <chrono>
#include <optional>
#include <string>

namespace cl_moveit2z
{
/**
 * @brief Configuration options for trajectory execution
 */
struct ExecutionOptions
{
  std::optional<double> maxVelocityScaling;
  std::optional<double> maxAccelerationScaling;
  bool recordToHistory = true;
  std::optional<std::string> trajectoryName;
};

/**
 * @brief Result of a trajectory execution operation
 */
struct ExecutionResult
{
  bool success;
  moveit_msgs::msg::MoveItErrorCodes errorCode;
  std::string errorMessage;
  std::chrono::duration<double> executionTime;

  ExecutionResult() : success(false), errorCode(), errorMessage(""), executionTime(0.0)
  {
    errorCode.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
};

/**
 * @brief Component for centralized trajectory execution
 *
 * This component provides a unified interface for executing trajectories with:
 * - Automatic recording to CpTrajectoryHistory
 * - Consistent error handling and event posting
 * - Execution options (velocity/acceleration scaling)
 * - Execution time tracking
 *
 * Pattern: Executor with signals (like action clients)
 */
class CpTrajectoryExecutor : public smacc2::ISmaccComponent
{
public:
  CpTrajectoryExecutor() = default;
  virtual ~CpTrajectoryExecutor() = default;

  /**
   * @brief Initialize the component and get references to required components/clients
   */
  inline void onInitialize() override
  {
    this->requiresClient(moveit2zClient_);

    // CpTrajectoryHistory is optional but recommended
    this->requiresComponent(trajectoryHistory_, false);

    if (trajectoryHistory_)
    {
      RCLCPP_INFO(
        getLogger(),
        "[CpTrajectoryExecutor] Component initialized with trajectory history recording enabled");
    }
    else
    {
      RCLCPP_WARN(
        getLogger(),
        "[CpTrajectoryExecutor] Component initialized without CpTrajectoryHistory "
        "(trajectory history will not be recorded)");
    }
  }

  /**
   * @brief Execute a trajectory synchronously
   *
   * @param trajectory The robot trajectory to execute
   * @param options Execution configuration options
   * @return ExecutionResult with success status and error information
   */
  inline ExecutionResult execute(
    const moveit_msgs::msg::RobotTrajectory & trajectory, const ExecutionOptions & options = {})
  {
    ExecutionResult result;

    if (!moveit2zClient_)
    {
      result.errorMessage = "ClMoveit2z client not available";
      RCLCPP_ERROR(getLogger(), "[CpTrajectoryExecutor] %s", result.errorMessage.c_str());
      return result;
    }

    auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

    try
    {
      // Apply execution options
      applyExecutionOptions(*moveGroup, options);

      RCLCPP_INFO(
        getLogger(), "[CpTrajectoryExecutor] Executing trajectory with %lu points...",
        trajectory.joint_trajectory.points.size());

      // Track execution time
      auto startTime = std::chrono::steady_clock::now();

      // Execute trajectory
      result.errorCode = moveGroup->execute(trajectory);

      auto endTime = std::chrono::steady_clock::now();
      result.executionTime = std::chrono::duration<double>(endTime - startTime);

      result.success = (result.errorCode.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      if (result.success)
      {
        RCLCPP_INFO(
          getLogger(), "[CpTrajectoryExecutor] Trajectory execution succeeded (%.2f seconds)",
          result.executionTime.count());
      }
      else
      {
        result.errorMessage =
          "Trajectory execution failed with error code: " + std::to_string(result.errorCode.val);
        RCLCPP_WARN(getLogger(), "[CpTrajectoryExecutor] %s", result.errorMessage.c_str());
      }

      // Record to history if enabled
      if (options.recordToHistory && trajectoryHistory_)
      {
        std::string trajName = options.trajectoryName.value_or("unnamed_trajectory");
        recordToHistory(trajectory, result.errorCode, trajName);
      }
    }
    catch (const std::exception & e)
    {
      result.success = false;
      result.errorMessage = std::string("Exception during trajectory execution: ") + e.what();
      RCLCPP_ERROR(getLogger(), "[CpTrajectoryExecutor] %s", result.errorMessage.c_str());
    }

    return result;
  }

  /**
   * @brief Execute a motion plan synchronously
   *
   * Convenience method that accepts a MoveGroupInterface::Plan directly
   *
   * @param plan The motion plan containing the trajectory
   * @param options Execution configuration options
   * @return ExecutionResult with success status and error information
   */
  inline ExecutionResult executePlan(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const ExecutionOptions & options = {})
  {
    return execute(plan.trajectory_, options);
  }

  /**
   * @brief Cancel ongoing execution
   *
   * Note: This is a best-effort cancellation and may not immediately stop the robot
   */
  inline void cancel()
  {
    if (!moveit2zClient_)
    {
      RCLCPP_ERROR(getLogger(), "[CpTrajectoryExecutor] Cannot cancel: client not available");
      return;
    }

    auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

    try
    {
      moveGroup->stop();
      RCLCPP_INFO(getLogger(), "[CpTrajectoryExecutor] Trajectory execution cancelled");
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(), "[CpTrajectoryExecutor] Exception during cancellation: %s", e.what());
    }
  }

  /**
   * @brief Get the trajectory history component
   *
   * @return Pointer to CpTrajectoryHistory, or nullptr if not available
   */
  inline CpTrajectoryHistory * getTrajectoryHistory() { return trajectoryHistory_; }

private:
  ClMoveit2z * moveit2zClient_ = nullptr;
  CpTrajectoryHistory * trajectoryHistory_ = nullptr;

  /**
   * @brief Apply execution options to MoveGroupInterface
   *
   * @param moveGroup Reference to MoveGroupInterface
   * @param options Execution options to apply
   */
  inline void applyExecutionOptions(
    moveit::planning_interface::MoveGroupInterface & moveGroup, const ExecutionOptions & options)
  {
    if (options.maxVelocityScaling)
    {
      moveGroup.setMaxVelocityScalingFactor(*options.maxVelocityScaling);
    }

    if (options.maxAccelerationScaling)
    {
      moveGroup.setMaxAccelerationScalingFactor(*options.maxAccelerationScaling);
    }
  }

  /**
   * @brief Record trajectory to history component
   *
   * @param trajectory The executed trajectory
   * @param errorCode Execution result error code
   * @param name Name for this trajectory entry
   */
  inline void recordToHistory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const moveit_msgs::msg::MoveItErrorCodes & errorCode, const std::string & name)
  {
    if (!trajectoryHistory_)
    {
      return;
    }

    try
    {
      trajectoryHistory_->pushTrajectory(name, trajectory, errorCode);
      RCLCPP_DEBUG(
        getLogger(), "[CpTrajectoryExecutor] Recorded trajectory '%s' to history", name.c_str());
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(
        getLogger(), "[CpTrajectoryExecutor] Failed to record trajectory to history: %s", e.what());
    }
  }
};

}  // namespace cl_moveit2z
