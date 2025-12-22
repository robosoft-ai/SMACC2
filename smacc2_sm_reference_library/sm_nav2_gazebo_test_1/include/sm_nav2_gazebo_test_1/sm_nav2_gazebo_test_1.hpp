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

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_nav2z/cl_nav2z.hpp>

// CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_global_position.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_transform.hpp>
#include <cl_nav2z/client_behaviors/cb_pure_spinning.hpp>
#include <cl_nav2z/components/amcl/cp_amcl.hpp>

// ORTHOGONALS
#include "orthogonals/or_timer.hpp"
#include "orthogonals/or_keyboard.hpp"
#include "orthogonals/or_navigation.hpp"

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace cl_nav2z;

namespace sm_nav2_gazebo_test_1
{

// FORWARD DECLARATIONS
class StAllSensorsGo;
class StSetInitialPose;
class StNavigateToWaypoint1;
class StRotate;
class StNavigateToWaypoint2;
class StFinalState;

//--------------------------------------------------------------------
// STATE MACHINE
struct SmNav2GazeboTest1 : public smacc2::SmaccStateMachineBase<SmNav2GazeboTest1, StAllSensorsGo>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrNavigation>();
  }
};

}  // namespace sm_nav2_gazebo_test_1

// STATE INCLUDES (must be after state machine definition)
#include "states/st_all_sensors_go.hpp"
#include "states/st_set_initial_pose.hpp"
#include "states/st_navigate_to_waypoint_1.hpp"
#include "states/st_rotate.hpp"
#include "states/st_navigate_to_waypoint_2.hpp"
#include "states/st_final_state.hpp"
