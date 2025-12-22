// Copyright 2024 RobosoftAI Inc.
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

#pragma once

#include <smacc2/smacc.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp>

namespace sm_nav2_gazebo_test_1
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// Custom transition tag for retry
struct RETRY : SUCCESS {};

// Custom event for max retries exceeded
template <typename TSource, typename TOrthogonal>
struct EvMaxRetriesExceeded : sc::event<EvMaxRetriesExceeded<TSource, TOrthogonal>>
{
};

// Forward declarations
struct StNavigateToWaypoint1;
struct StFinalState;

// STATE DECLARATION
struct StSetInitialPose : smacc2::SmaccState<StSetInitialPose, SmNav2GazeboTest1>
{
  using SmaccState::SmaccState;

  // Retry configuration: MAX_RETRIES means number of retries after initial attempt
  // Total attempts = 1 (initial) + MAX_RETRIES = 4
  static constexpr int MAX_RETRIES = 3;
  static constexpr int MAX_ATTEMPTS = 1 + MAX_RETRIES;  // 4 total attempts
  static constexpr const char* ATTEMPT_COUNTER_KEY = "localization_attempt_count";

  // TRANSITION TABLE
  typedef mpl::list<
    // Success: proceed to navigation when Nav2 nodes are ready
    Transition<EvCbSuccess<CbWaitNav2Nodes, OrNavigation>, StNavigateToWaypoint1, SUCCESS>,
    // Failure with retries remaining: self-transition to try again
    Transition<EvCbFailure<CbWaitNav2Nodes, OrNavigation>, StSetInitialPose, RETRY>,
    // Max retries exceeded: abort to final state
    Transition<EvMaxRetriesExceeded<CbWaitNav2Nodes, OrNavigation>, StFinalState, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Wait for Nav2 nodes to be active (PlannerServer, ControllerServer, BtNavigator)
    // This ensures the navigation stack is ready before attempting to navigate
    configure_orthogonal<OrNavigation, CbWaitNav2Nodes>();

    // Keyboard behavior for manual control
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    // Increment attempt counter first
    int attemptCount = 0;
    this->getGlobalSMData(ATTEMPT_COUNTER_KEY, attemptCount);
    attemptCount++;
    this->setGlobalSMData(ATTEMPT_COUNTER_KEY, attemptCount);

    // Check if max attempts exceeded (1 initial + MAX_RETRIES)
    if (attemptCount > MAX_ATTEMPTS)
    {
      RCLCPP_ERROR(
        getLogger(),
        "StSetInitialPose: All %d attempts failed, aborting mission", MAX_ATTEMPTS);
      // Post abort event - this will be processed and transition to StFinalState
      this->postEvent<EvMaxRetriesExceeded<CbWaitNav2Nodes, OrNavigation>>();
      return;
    }

    // Log current attempt
    if (attemptCount == 1)
    {
      RCLCPP_INFO(getLogger(), "StSetInitialPose: Initial attempt (1 of %d)", MAX_ATTEMPTS);
    }
    else
    {
      RCLCPP_WARN(
        getLogger(),
        "StSetInitialPose: Retry %d of %d (attempt %d of %d)",
        attemptCount - 1, MAX_RETRIES, attemptCount, MAX_ATTEMPTS);
    }

    RCLCPP_INFO(getLogger(), "StSetInitialPose: runtimeConfigure() - Setting initial pose for AMCL");

    // Set initial pose for AMCL localization
    // Robot spawns at x=-2.0, y=-0.5 in tb3_simulation_launch.py
    cl_nav2z::ClNav2Z* navClient;
    requiresClient(navClient);

    cl_nav2z::CpAmcl* amclComponent = navClient->getComponent<cl_nav2z::CpAmcl>();

    geometry_msgs::msg::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "map";
    initialPose.header.stamp = getNode()->now();
    initialPose.pose.pose.position.x = -2.0;
    initialPose.pose.pose.position.y = -0.5;
    initialPose.pose.pose.position.z = 0.0;
    initialPose.pose.pose.orientation.x = 0.0;
    initialPose.pose.pose.orientation.y = 0.0;
    initialPose.pose.pose.orientation.z = 0.0;
    initialPose.pose.pose.orientation.w = 1.0;
    // Set covariance (small values for good initial estimate)
    initialPose.pose.covariance[0] = 0.25;   // x variance
    initialPose.pose.covariance[7] = 0.25;   // y variance
    initialPose.pose.covariance[35] = 0.06;  // yaw variance

    amclComponent->setInitialPose(initialPose);
    RCLCPP_INFO(getLogger(), "StSetInitialPose: Initial pose published (x=-2.0, y=-0.5), waiting for transform...");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StSetInitialPose: onEntry() - Waiting for Nav2 nodes to be active");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StSetInitialPose: onExit()");
  }

  // Reset attempt counter on successful exit to navigation
  void onExit(SUCCESS)
  {
    // Clear attempt counter on success
    this->setGlobalSMData(ATTEMPT_COUNTER_KEY, 0);
    RCLCPP_INFO(getLogger(), "StSetInitialPose: Localization successful, proceeding to navigation");
  }

  void onExit(ABORT)
  {
    RCLCPP_ERROR(getLogger(), "StSetInitialPose: Localization failed after max retries, aborting");
  }
};

}  // namespace sm_nav2_gazebo_test_1
