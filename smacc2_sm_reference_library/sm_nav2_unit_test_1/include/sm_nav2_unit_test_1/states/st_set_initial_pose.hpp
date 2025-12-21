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
#include <cl_nav2z/client_behaviors/cb_wait_transform.hpp>

namespace sm_nav2_unit_test_1
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// Forward declaration
struct StNavigateToWaypoint1;

// STATE DECLARATION
struct StSetInitialPose : smacc2::SmaccState<StSetInitialPose, SmNav2UnitTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitTransform, OrNavigation>, StNavigateToWaypoint1, SUCCESS>,
    Transition<EvCbFailure<CbWaitTransform, OrNavigation>, StNavigateToWaypoint1, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Wait for map->base_link transform (10 second timeout)
    configure_orthogonal<OrNavigation, CbWaitTransform>("base_link", "map", rclcpp::Duration(10, 0));

    // Keyboard behavior for manual control
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
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
    RCLCPP_INFO(getLogger(), "StSetInitialPose: onEntry() - Waiting for map->base_link transform");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StSetInitialPose: onExit() - Localization ready");
  }
};

}  // namespace sm_nav2_unit_test_1
