// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

namespace sm_test_moveit_ur5_sim
{
// SMACC2 classes
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using namespace smacc2;

// STATE DECLARATION
struct StMoveEndEffector : smacc2::SmaccState<StMoveEndEffector, SmTestMoveitUr5Sim>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
    Transition<EvCbSuccess<CbMoveEndEffector, OrArm>, StEndEffectorRotate, SUCCESS>,
    Transition<EvCbFailure<CbMoveEndEffector, OrArm>, StMoveEndEffector, ABORT>
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    target_pose.pose.position.x = 0.137;
    target_pose.pose.position.y = -0.630;
    target_pose.pose.position.z = 0.467;

    target_pose.pose.orientation.x = 0.498;
    target_pose.pose.orientation.y = 0.470;
    target_pose.pose.orientation.z = 0.715;

    target_pose.pose.orientation.w = 0.141;

    //target_pose.pose.position.z = 0.579;

  // random valid
  // - Translation: [0.137, -0.730, 0.267]
  // - Rotation: in Quaternion [0.498, 0.470, 0.715, 0.141]


    //0.8939967, 0, 0, -0.4480736 ]
    // target_pose.pose.orientation.x = 0.8939967;
    // target_pose.pose.orientation.y = 0;
    // target_pose.pose.orientation.z = 0;
    // target_pose.pose.orientation.w = -0.4480736 ;

    configure_orthogonal<OrArm, CbMoveEndEffector>(target_pose, "tool0");
  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "Entering StMoveEndEffector"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_test_moveit_ur5_sim
