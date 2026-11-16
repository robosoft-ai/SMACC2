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

namespace sm_nav2_gazebo_test_4
{
// Publishes the AMCL initial pose and waits with CbWaitPose until the pose component has a
// map pose. AMCL drops a pose published before it is active, so the timer re-enters the state,
// which publishes it again.
struct StSetInitialPose : smacc2::SmaccState<StSetInitialPose, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitPose, OrNavigation>, StAcquireSensors, SUCCESS>,
    Transition<EvCbFailure<CbWaitPose, OrNavigation>, StMissionFailed, ABORT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StSetInitialPose, RETRY>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAcquireSensors, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbWaitPose>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    cl_nav2z::ClNav2Z * navClient = nullptr;
    requiresClient(navClient);
    auto * amcl = navClient->getComponent<cl_nav2z::CpAmcl>();

    // the robot spawns at (-2.0, -0.5) facing +x in tb3_simulation_launch.py
    geometry_msgs::msg::PoseWithCovarianceStamped initialPose;
    initialPose.header.frame_id = "map";
    initialPose.header.stamp = getNode()->now();
    initialPose.pose.pose.position.x = -2.0;
    initialPose.pose.pose.position.y = -0.5;
    initialPose.pose.pose.orientation.w = 1.0;
    initialPose.pose.covariance[0] = 0.25;
    initialPose.pose.covariance[7] = 0.25;
    initialPose.pose.covariance[35] = 0.06;
    amcl->setInitialPose(initialPose);
    RCLCPP_INFO(getLogger(), "StSetInitialPose: initial pose published (-2.0, -0.5)");
  }
};

}  // namespace sm_nav2_gazebo_test_4
