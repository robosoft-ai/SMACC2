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
struct StPouringMotion : smacc2::SmaccState<StPouringMotion, SmTestMoveitUr5Sim>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
      Transition<EvCbSuccess<CbCircularPouringMotion, OrArm>, StMoveLastTrajectoryInitialState, SUCCESS>,
      Transition<EvCbFailure<CbCircularPouringMotion, OrArm>, StMoveLastTrajectoryInitialState, ABORT>
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
   geometry_msgs::msg::Point relativePivotPoint;
   relativePivotPoint.x = -0.01;
   double deltaHeight = 0.05;
   std::string tipLink = "tool0";
   std::string globalFrame = "tool0";

    configure_orthogonal<OrArm, CbCircularPouringMotion>(relativePivotPoint, deltaHeight, tipLink, globalFrame);
  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "Entering StPouringMotion"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_test_moveit_ur5_sim
