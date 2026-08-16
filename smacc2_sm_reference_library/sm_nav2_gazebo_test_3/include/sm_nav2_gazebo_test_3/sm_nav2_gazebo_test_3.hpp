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

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_nav2z/cl_nav2z.hpp>

// CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp>
#include <cl_nav2z/client_behaviors/cb_spin.hpp>
#include <cl_nav2z/client_behaviors/cb_back_up.hpp>
#include <cl_nav2z/client_behaviors/cb_drive_on_heading.hpp>
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

namespace sm_nav2_gazebo_test_3
{

// STATE FORWARD DECLARATIONS
class StAllSensorsGo;
class StSetInitialPose;
class StSpinToWall;
class StDriveAtWall;
class StBackUpSafe;
class StFinalState;

// SUPERSTATE FORWARD DECLARATIONS
namespace SS1
{
class SsPrimitiveLoop;
}

//--------------------------------------------------------------------
// STATE MACHINE
//
// Behavior-server primitives mission: after bring-up, a superstate loops
// DriveOnHeading -> Spin -> BackUp using the Nav2 behavior server actions
// through CbActionClientBehaviorBase - no planner/controller stack involved.
struct SmNav2GazeboTest3 : public smacc2::SmaccStateMachineBase<SmNav2GazeboTest3, StAllSensorsGo>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrNavigation>();
  }
};

}  // namespace sm_nav2_gazebo_test_3

// SUPERSTATES (must be after state machine definition)
#include "superstates/ss_primitive_loop.hpp"

// STATE INCLUDES (must be after state machine definition)
#include "states/st_all_sensors_go.hpp"
#include "states/st_set_initial_pose.hpp"
#include "states/st_spin_to_wall.hpp"
#include "states/st_drive_at_wall.hpp"
#include "states/st_back_up_safe.hpp"
#include "states/st_final_state.hpp"
