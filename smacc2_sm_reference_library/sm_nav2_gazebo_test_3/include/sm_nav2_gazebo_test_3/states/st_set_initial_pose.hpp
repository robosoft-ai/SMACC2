// Copyright 2026 RobosoftAI Inc.
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

namespace sm_nav2_gazebo_test_3
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;

// Custom transition tag for retry
struct RETRY : SUCCESS {};

// Custom event for max retries exceeded
template <typename TSource, typename TOrthogonal>
struct EvMaxRetriesExceeded : sc::event<EvMaxRetriesExceeded<TSource, TOrthogonal>>
{
};

// STATE DECLARATION
struct StSetInitialPose : smacc2::SmaccState<StSetInitialPose, SmNav2GazeboTest3>
{
  using SmaccState::SmaccState;

  // Retry configuration: total attempts = 1 (initial) + MAX_RETRIES
  static constexpr int MAX_RETRIES = 3;
  static constexpr int MAX_ATTEMPTS = 1 + MAX_RETRIES;
  static constexpr const char * ATTEMPT_COUNTER_KEY = "localization_attempt_count";

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitNav2Nodes, OrNavigation>, SS1::SsPrimitiveLoop, SUCCESS>,
    Transition<EvCbFailure<CbWaitNav2Nodes, OrNavigation>, StSetInitialPose, RETRY>,
    Transition<EvMaxRetriesExceeded<CbWaitNav2Nodes, OrNavigation>, StFinalState, ABORT>,
    // Fallbacks: CbWaitNav2Nodes sniffs /bond heartbeats and can miss them on a
    // busy bond topic even when Nav2 is fully active - see StAllSensorsGo
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SS1::SsPrimitiveLoop, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::SsPrimitiveLoop, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Wait for Nav2 nodes to be active before attempting to navigate
    configure_orthogonal<OrNavigation, CbWaitNav2Nodes>();

    // Timer fallback: by this point Nav2 activation has already been observed
    // by StAllSensorsGo; do not block forever on missed bond heartbeats
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(20s);

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

    if (attemptCount > MAX_ATTEMPTS)
    {
      RCLCPP_ERROR(
        getLogger(), "StSetInitialPose: All %d attempts failed, aborting mission", MAX_ATTEMPTS);
      this->postEvent<EvMaxRetriesExceeded<CbWaitNav2Nodes, OrNavigation>>();
      return;
    }

    if (attemptCount == 1)
    {
      RCLCPP_INFO(getLogger(), "StSetInitialPose: Initial attempt (1 of %d)", MAX_ATTEMPTS);
    }
    else
    {
      RCLCPP_WARN(
        getLogger(), "StSetInitialPose: Retry %d of %d (attempt %d of %d)", attemptCount - 1,
        MAX_RETRIES, attemptCount, MAX_ATTEMPTS);
    }

    RCLCPP_INFO(getLogger(), "StSetInitialPose: Setting initial pose for AMCL");

    // Robot spawns at x=-2.0, y=-0.5 in tb3_simulation_launch.py
    cl_nav2z::ClNav2Z * navClient;
    requiresClient(navClient);

    cl_nav2z::CpAmcl * amclComponent = navClient->getComponent<cl_nav2z::CpAmcl>();

    geometry_msgs::msg::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "map";
    initialPose.header.stamp = getNode()->now();
    initialPose.pose.pose.position.x = -2.0;
    initialPose.pose.pose.position.y = -0.5;
    initialPose.pose.pose.position.z = 0.0;
    initialPose.pose.pose.orientation.w = 1.0;
    initialPose.pose.covariance[0] = 0.25;   // x variance
    initialPose.pose.covariance[7] = 0.25;   // y variance
    initialPose.pose.covariance[35] = 0.06;  // yaw variance

    amclComponent->setInitialPose(initialPose);
    RCLCPP_INFO(
      getLogger(),
      "StSetInitialPose: Initial pose published (x=-2.0, y=-0.5), waiting for transform...");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StSetInitialPose: onEntry() - Waiting for Nav2 nodes to be active");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StSetInitialPose: onExit()");
  }

  void onExit(SUCCESS)
  {
    this->setGlobalSMData(ATTEMPT_COUNTER_KEY, 0);
    RCLCPP_INFO(getLogger(), "StSetInitialPose: Localization successful, proceeding to navigation");
  }

  void onExit(ABORT)
  {
    RCLCPP_ERROR(getLogger(), "StSetInitialPose: Localization failed after max retries, aborting");
  }
};

}  // namespace sm_nav2_gazebo_test_3
