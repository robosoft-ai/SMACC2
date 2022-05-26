// Copyright 2021 RobosoftAI Inc.
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

// CLIENT BEHAVIORS
#include <nav2z_client/nav2z_client.hpp>
#include <nav2z_client/client_behaviors.hpp>
#include <sm_husky_barrel_search_1/clients/cb_sleep_for.hpp>

// ORTHOGONALS
#include "orthogonals/or_navigation.hpp"
#include "orthogonals/or_perception.hpp"
#include "orthogonals/or_led_array.hpp"

namespace sm_husky_barrel_search_1
{

//STATES
struct StAcquireSensors;
struct StDetectItems;
struct StNavigateToWaypointX;
struct StDeactivateMine;
struct StSatelliteCommunications;
struct StSelfDestruction;
struct StExploreAndRetreat;
struct StUndoRetreat;
struct StMoveBackwardsBlinking;
struct StCrossMineFieldSlowly;

namespace SS5
{
class SsSearchMineSPattern1;
}

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmHuskyBarrelSearch1
: public smacc2::SmaccStateMachineBase<SmHuskyBarrelSearch1, StAcquireSensors>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrNavigation>();
    this->createOrthogonal<OrPerception>();
    this->createOrthogonal<OrLedArray>();
  }
};
}  // namespace sm_husky_barrel_search_1

//STATES
#include "states/st_acquire_sensors.hpp"
#include "states/st_detect_items.hpp"
#include "states/st_navigate_to_waypoint_x.hpp"
#include "states/st_deactivate_mine.hpp"
#include "states/st_self_destruction.hpp"
#include "states/st_satellite_communications.hpp"
#include "states/st_evasion_motion.hpp"
#include "superstates/ss_search_mine_s_pattern_1.hpp"
#include "states/st_explore_and_retreat.hpp"
#include "states/st_undo_retreat.hpp"
#include "states/st_move_backwards_blinking.hpp"
#include "states/st_cross_mine_field_slowly.hpp"
