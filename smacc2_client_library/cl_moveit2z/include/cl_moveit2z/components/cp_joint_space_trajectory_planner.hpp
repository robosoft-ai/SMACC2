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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace cl_moveit2z
{
/**
 * @brief Error codes for joint space trajectory computation
 */
enum class JointTrajectoryErrorCode
{
  SUCCESS,
  INCORRECT_INITIAL_STATE,
  JOINT_TRAJECTORY_DISCONTINUITY,
  IK_SOLUTION_FAILED,
  EMPTY_WAYPOINT_LIST
};

/**
 * @brief Configuration options for joint space trajectory planning
 */
struct JointTrajectoryOptions
{
  std::optional<std::string> tipLink;
  std::optional<std::string> groupName;
  bool allowInitialDiscontinuity = false;
  double maxJointDiscontinuity = 0.3;  // radians (~17 degrees)
  int ikAttempts = 4;
  bool avoidCollisions = true;
  std::chrono::seconds ikTimeout = 3s;
};

/**
 * @brief Result of a joint space trajectory planning operation
 */
struct JointTrajectoryResult
{
  bool success;
  JointTrajectoryErrorCode errorCode;
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::string errorMessage;
  std::vector<int> discontinuityIndexes;

  JointTrajectoryResult()
  : success(false), errorCode(JointTrajectoryErrorCode::IK_SOLUTION_FAILED), errorMessage("")
  {
  }
};

/**
 * @brief Component for joint space trajectory generation from Cartesian waypoints
 *
 * This component centralizes IK-based trajectory generation:
 * - Computes IK solutions for Cartesian waypoint sequences
 * - Validates trajectory continuity
 * - Detects and reports joint discontinuities
 * - Handles initial state validation
 *
 * Pattern: Trajectory planner with IK service integration
 */
class CpJointSpaceTrajectoryPlanner : public smacc2::ISmaccComponent
{
public:
  CpJointSpaceTrajectoryPlanner() = default;
  virtual ~CpJointSpaceTrajectoryPlanner() = default;

  /**
   * @brief Initialize the component and create IK service client
   */
  inline void onInitialize() override
  {
    this->requiresClient(moveit2zClient_);

    // Create IK service client
    ikClient_ = getNode()->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    RCLCPP_INFO(getLogger(), "[CpJointSpaceTrajectoryPlanner] Component initialized");
  }

  /**
   * @brief Compute joint space trajectory from Cartesian waypoints
   *
   * @param waypoints Vector of Cartesian poses to follow
   * @param options Planning configuration options
   * @return JointTrajectoryResult with success status and computed trajectory
   */
  inline JointTrajectoryResult planFromWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    const JointTrajectoryOptions & options = {})
  {
    JointTrajectoryResult result;

    if (waypoints.empty())
    {
      result.errorCode = JointTrajectoryErrorCode::EMPTY_WAYPOINT_LIST;
      result.errorMessage = "No waypoints provided";
      RCLCPP_WARN(getLogger(), "[CpJointSpaceTrajectoryPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    if (!moveit2zClient_)
    {
      result.errorMessage = "ClMoveit2z client not available";
      RCLCPP_ERROR(getLogger(), "[CpJointSpaceTrajectoryPlanner] %s", result.errorMessage.c_str());
      return result;
    }

    try
    {
      auto & moveGroup = moveit2zClient_->moveGroupClientInterface;

      // Get current robot state
      RCLCPP_INFO(
        getLogger(),
        "[CpJointSpaceTrajectoryPlanner] Getting current state for trajectory planning");
      auto currentState = moveGroup->getCurrentState(100);
      if (!currentState)
      {
        result.errorMessage = "Failed to get current robot state";
        RCLCPP_ERROR(
          getLogger(), "[CpJointSpaceTrajectoryPlanner] %s", result.errorMessage.c_str());
        return result;
      }

      // Determine group name and tip link
      std::string groupName = options.groupName.value_or(moveGroup->getName());
      std::string tipLink = options.tipLink.value_or(moveGroup->getEndEffectorLink());

      RCLCPP_INFO(
        getLogger(), "[CpJointSpaceTrajectoryPlanner] Planning for group '%s' with tip link '%s'",
        groupName.c_str(), tipLink.c_str());

      // Get joint names
      auto jointNames = currentState->getJointModelGroup(groupName)->getActiveJointModelNames();

      // Initialize trajectory with current state
      std::vector<double> jointPositions;
      currentState->copyJointGroupPositions(groupName, jointPositions);

      std::vector<std::vector<double>> trajectory;
      std::vector<rclcpp::Duration> trajectoryTimeStamps;

      trajectory.push_back(jointPositions);
      trajectoryTimeStamps.push_back(rclcpp::Duration(0s));

      // Reference time for trajectory timestamps
      auto & first = waypoints.front();
      rclcpp::Time referenceTime(first.header.stamp);

      // Compute IK for each waypoint
      int ikAttemptsRemaining = options.ikAttempts;
      for (size_t k = 0; k < waypoints.size(); k++)
      {
        auto & pose = waypoints[k];

        // Create IK request
        auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        req->ik_request.ik_link_name = tipLink;
        req->ik_request.robot_state.joint_state.name = jointNames;
        req->ik_request.robot_state.joint_state.position = jointPositions;
        req->ik_request.group_name = groupName;
        req->ik_request.avoid_collisions = options.avoidCollisions;
        req->ik_request.pose_stamped = pose;

        RCLCPP_DEBUG(
          getLogger(), "[CpJointSpaceTrajectoryPlanner] Computing IK for waypoint %lu/%lu", k + 1,
          waypoints.size());

        // Call IK service
        auto resFuture = ikClient_->async_send_request(req);
        auto status = resFuture.wait_for(options.ikTimeout);

        if (status != std::future_status::ready)
        {
          result.errorMessage = "IK service timeout at waypoint " + std::to_string(k);
          RCLCPP_ERROR(
            getLogger(), "[CpJointSpaceTrajectoryPlanner] %s", result.errorMessage.c_str());
          result.errorCode = JointTrajectoryErrorCode::IK_SOLUTION_FAILED;
          return result;
        }

        auto res = resFuture.get();
        auto & prevTrajPoint = trajectory.back();

        // Extract joint positions from IK solution
        for (size_t j = 0; j < res->solution.joint_state.position.size(); j++)
        {
          auto & jointName = res->solution.joint_state.name[j];
          auto it = std::find(jointNames.begin(), jointNames.end(), jointName);
          if (it != jointNames.end())
          {
            int index = std::distance(jointNames.begin(), it);
            jointPositions[index] = res->solution.joint_state.position[j];
          }
        }

        // Continuity check
        bool shouldCheck = k > 0 || !options.allowInitialDiscontinuity;
        int discontinuityJointIndex = -1;
        double discontinuityDelta = -1;

        if (shouldCheck)
        {
          for (size_t jointIndex = 0; jointIndex < jointPositions.size(); jointIndex++)
          {
            double deltaJoint = jointPositions[jointIndex] - prevTrajPoint[jointIndex];
            if (fabs(deltaJoint) > options.maxJointDiscontinuity)
            {
              discontinuityDelta = deltaJoint;
              discontinuityJointIndex = jointIndex;
              break;
            }
          }
        }

        // Handle discontinuity
        if (ikAttemptsRemaining > 0 && discontinuityJointIndex != -1)
        {
          // Retry IK
          k--;
          ikAttemptsRemaining--;
          RCLCPP_WARN(
            getLogger(),
            "[CpJointSpaceTrajectoryPlanner] IK discontinuity detected, retrying (%d attempts "
            "remaining)",
            ikAttemptsRemaining);
          continue;
        }
        else
        {
          // Record discontinuity or accept trajectory point
          if (ikAttemptsRemaining == 0 && discontinuityJointIndex != -1)
          {
            result.discontinuityIndexes.push_back(k);

            RCLCPP_ERROR(
              getLogger(),
              "[CpJointSpaceTrajectoryPlanner] Joint discontinuity at waypoint %lu: joint '%s' "
              "delta=%.3f rad",
              k, jointNames[discontinuityJointIndex].c_str(), discontinuityDelta);

            if (k == 0)
            {
              RCLCPP_ERROR(
                getLogger(),
                "[CpJointSpaceTrajectoryPlanner] Initial posture discontinuity detected");
            }
          }

          // Reset retry counter
          ikAttemptsRemaining = options.ikAttempts;

          // Add point to trajectory
          trajectory.push_back(jointPositions);
          rclcpp::Duration durationFromStart = rclcpp::Time(pose.header.stamp) - referenceTime;
          trajectoryTimeStamps.push_back(durationFromStart);
        }
      }

      // Build RobotTrajectory message
      result.trajectory.joint_trajectory.joint_names = jointNames;

      // Skip first point (current state) to avoid discontinuity in some behaviors
      for (size_t i = 1; i < trajectory.size(); i++)
      {
        trajectory_msgs::msg::JointTrajectoryPoint jp;
        jp.positions = trajectory[i];
        jp.time_from_start = trajectoryTimeStamps[i];
        result.trajectory.joint_trajectory.points.push_back(jp);
      }

      // Determine success
      if (!result.discontinuityIndexes.empty())
      {
        if (result.discontinuityIndexes[0] == 0)
        {
          result.errorCode = JointTrajectoryErrorCode::INCORRECT_INITIAL_STATE;
          result.errorMessage = "Incorrect initial state";
        }
        else
        {
          result.errorCode = JointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY;
          result.errorMessage = "Joint trajectory discontinuity detected";
        }
        result.success = false;
        RCLCPP_WARN(getLogger(), "[CpJointSpaceTrajectoryPlanner] %s", result.errorMessage.c_str());
      }
      else
      {
        result.success = true;
        result.errorCode = JointTrajectoryErrorCode::SUCCESS;
        RCLCPP_INFO(
          getLogger(),
          "[CpJointSpaceTrajectoryPlanner] Successfully computed trajectory with %lu points",
          result.trajectory.joint_trajectory.points.size());
      }
    }
    catch (const std::exception & e)
    {
      result.success = false;
      result.errorMessage = std::string("Exception during trajectory planning: ") + e.what();
      RCLCPP_ERROR(getLogger(), "[CpJointSpaceTrajectoryPlanner] %s", result.errorMessage.c_str());
    }

    return result;
  }

private:
  ClMoveit2z * moveit2zClient_ = nullptr;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ikClient_;
};

}  // namespace cl_moveit2z
