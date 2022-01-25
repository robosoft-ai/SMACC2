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

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// ORTHOGONALS
#include "orthogonals/or_autoware_auto.hpp"
#include "orthogonals/or_timer.hpp"

// CLIENTS
#include "clients/autoware_client/cl_autoware.hpp"

namespace sm_autoware_avp
{
// SMACC2 clases
using sm_autoware_avp::OrAutowareAuto;

//STATES
struct StAcquireSensors;
struct StSetupInitialLocationEstimation;
struct StNavigateWaypoint1;
struct StNavigateWaypoint2;
struct StFirstPause;
struct StSecondPause;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAutowareAvp
: public smacc2::SmaccStateMachineBase<SmAutowareAvp, StAcquireSensors>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrAutowareAuto>();
    this->createOrthogonal<OrTimer>();
  }
};

}  // namespace sm_autoware_avp

// STATES
#include "states/st_acquire_sensors.hpp"
#include "states/st_setup_initial_location_estimation.hpp"
#include "states/st_navigate_waypoint_1.hpp"
#include "states/st_navigate_waypoint_2.hpp"
#include "states/st_first_pause.hpp"
#include "states/st_second_pause.hpp"
