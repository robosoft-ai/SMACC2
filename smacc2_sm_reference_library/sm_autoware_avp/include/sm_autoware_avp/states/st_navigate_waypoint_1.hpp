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
 ******************************************************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// CLIENTS
#include "ros_timer_client/cl_ros_timer.hpp"
#include "ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp"
#include "ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp"

// ORTHOGONALS
#include "sm_autoware_avp/orthogonals/or_autoware_auto.hpp"

namespace sm_autoware_avp
{
// SMACC2 clases
using sm_autoware_avp::clients::autoware_client::CbNavigateGlobalPosition;
using namespace sm_autoware_avp::clients;

using smacc2::EvStateRequestFinish;
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using smacc2::default_transition_tags::DEFAULT;

struct REPLAN:  smacc2::default_transition_tags::CONTINUELOOP { };

// STATE DECLARATION
struct StNavigateWaypoint1 : smacc2::SmaccState<StNavigateWaypoint1, SmAutowareAvp>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<

      // smacc2::EvCbSuccess<CbNavigateGlobalPosition, OrAutowareAuto>, StNavigateWaypoint2, SUCCESS>,
      Transition<EvGoalReached<ClAutoware, OrAutowareAuto>, StFirstPause, SUCCESS>,
      Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StNavigateWaypoint1, REPLAN>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    geometry_msgs::msg::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.pose.position.x = -45.253597259521484;
    goal.pose.position.y = 93.64368438720703;
    goal.pose.position.z = 0;

    goal.pose.orientation.z = 0.34841036522579555;
    goal.pose.orientation.w = 0.9373421026515494;

    configure_orthogonal<OrAutowareAuto, CbNavigateGlobalPosition>(goal);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);
  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "Entering StNavigateWaypoint1"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_autoware_avp
