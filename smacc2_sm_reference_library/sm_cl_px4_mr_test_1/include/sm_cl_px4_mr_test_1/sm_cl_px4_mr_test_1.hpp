// Copyright 2025 Robosoft Inc.
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
#include <cl_px4_mr/cl_px4_mr.hpp>
#include <cl_ros2_timer/cl_ros2_timer.hpp>

// CLIENT BEHAVIORS
#include <cl_px4_mr/client_behaviors/cb_arm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_disarm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_takeoff.hpp>
#include <cl_px4_mr/client_behaviors/cb_land.hpp>
#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <cl_px4_mr/client_behaviors/cb_orbit_location.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

// ORTHOGONALS
#include <sm_cl_px4_mr_test_1/orthogonals/or_px4.hpp>
#include <sm_cl_px4_mr_test_1/orthogonals/or_timer.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_cl_px4_mr_test_1
{

// MODE STATES (forward declarations)
class MsDisarmedOnGround;
class MsArmedOnGround;
class MsTakeoff;
class MsInFlight;
class MsLanding;
class MsLanded;

// STATES (forward declarations)
class StWaitForReady;
class StArmPx4;
class StTakeoff;
class StGoToWaypoint1;
class StOrbitLocation;
class StReturnToBase;
class StLand;
class StLanded;

// STATE MACHINE
struct SmClPx4MrTest1
: public smacc2::SmaccStateMachineBase<SmClPx4MrTest1, MsDisarmedOnGround>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrPx4>();
    this->createOrthogonal<OrTimer>();
  }
};

}  // namespace sm_cl_px4_mr_test_1

// MODE STATE INCLUDES (must come after SM definition, before regular states)
#include <sm_cl_px4_mr_test_1/states/ms_disarmed_on_ground.hpp>
#include <sm_cl_px4_mr_test_1/states/ms_armed_on_ground.hpp>
#include <sm_cl_px4_mr_test_1/states/ms_takeoff.hpp>
#include <sm_cl_px4_mr_test_1/states/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_1/states/ms_landing.hpp>
#include <sm_cl_px4_mr_test_1/states/ms_landed.hpp>

// REGULAR STATE INCLUDES (must come after mode state definitions)
#include <sm_cl_px4_mr_test_1/states/st_wait_for_ready.hpp>
#include <sm_cl_px4_mr_test_1/states/st_arm_px4.hpp>
#include <sm_cl_px4_mr_test_1/states/st_takeoff.hpp>
#include <sm_cl_px4_mr_test_1/states/st_go_to_waypoint_1.hpp>
#include <sm_cl_px4_mr_test_1/states/st_orbit_location.hpp>
#include <sm_cl_px4_mr_test_1/states/st_return_to_base.hpp>
#include <sm_cl_px4_mr_test_1/states/st_land.hpp>
#include <sm_cl_px4_mr_test_1/states/st_landed.hpp>
