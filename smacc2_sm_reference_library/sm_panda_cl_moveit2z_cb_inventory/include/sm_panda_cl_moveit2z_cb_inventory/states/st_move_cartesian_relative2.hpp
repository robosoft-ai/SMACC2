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

namespace sm_panda_cl_moveit2z_cb_inventory
{
// SMACC2 classes
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using namespace smacc2;
using namespace cl_keyboard;

// STATE DECLARATION
struct StMoveCartesianRelative2 : smacc2::SmaccState<StMoveCartesianRelative2, SmPandaClMoveit2zCbInventory>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef boost::mpl::list<
   Transition<EvCbSuccess<CbMoveCartesianRelative2, OrArm>, StPause6, SUCCESS>,
   // Transition<EvCbSuccess<CbMoveCartesianRelative2, OrArm>, StAttachObject, SUCCESS>,
   // Transition<EvCbFailure<CbMoveCartesianRelative2, OrArm>, StMoveCartesianRelative2, ABORT>, // retry

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StPause6, NEXT>  
    >

    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    geometry_msgs::msg::Vector3 position;
    position.x = -0.25;  // Increased to 25cm for dramatic movement
    position.y = 0.0;
    position.z = 0.20;   // Increased to 20cm for dramatic movement
    configure_orthogonal<OrArm, CbMoveCartesianRelative2>("world", "panda_link8", position);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "Entering StMoveCartesianRelative2"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit(ABORT)
  {
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(getLogger(), "On Exit!");
  }
};
}  // namespace sm_panda_cl_moveit2z_cb_inventory
