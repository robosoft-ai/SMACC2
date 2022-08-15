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
struct StExplore1;
struct StExplore2;
struct StExplore3;
struct StExplore4;
struct StExplore5;
struct StExplore6;
struct StExplore7;
struct StExplore8;
struct StAirStrikeCommunications;
struct StUndoRetreat;
struct StEvasionMotion;

struct StNavigatePrebarriers;
struct StNavigateBarriersForwardNext;

struct StMoveBackwardsBlinking;
struct StCrossMineFieldSlowly;
struct StLedBlinkingCommuncation;
struct StRetreatMotionFromMine;
struct StSelectSaferMinePath;

struct StBackupFromParking;
struct StExitBase;
struct StForwardAwayBase;

struct StNavigatePrebarriers;

struct StNavigateToFireEnemyPosition;
struct StFire;
struct StMissionAccomplished;
struct StMoveBaseEntrance;

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

#include "superstates/ss_search_mine_s_pattern_1.hpp"

#include "states/explore_left/st_explore_1.hpp"
#include "states/explore_left/st_explore_2.hpp"
#include "states/explore_left/st_explore_3.hpp"
#include "states/explore_left/st_explore_4.hpp"
#include "states/explore_left/st_explore_5.hpp"
#include "states/explore_left/st_explore_6.hpp"
#include "states/explore_left/st_explore_7.hpp"
#include "states/explore_left/st_explore_8.hpp"

#include "states/explore_left/st_undo_retreat.hpp"
#include "states/explore_left/st_airstrike_communications.hpp"
#include "states/explore_left/st_evasion_motion.hpp"

#include "states/base_sequence/st_backup_from_parking.hpp"
#include "states/base_sequence/st_move_base_entrance.hpp"
#include "states/base_sequence/st_led_blinking_communication.hpp"
#include "states/base_sequence/st_exit_base.hpp"
#include "states/base_sequence/st_forward_away_base.hpp"

#include "states/mine_detection_retreat/st_navigate_prebarriers.hpp"
#include "states/mine_detection_retreat/st_navigate_barriers_forward_next.hpp"
#include "states/mine_detection_retreat/st_retreat_motion_from_mine.hpp"
#include "states/mine_detection_retreat/st_select_safer_mine_path.hpp"
#include "states/mine_detection_retreat/st_cross_mine_field_slowly.hpp"

#include "states/attack_enemy/st_navigate_to_fire_enemy_position.hpp"
#include "states/attack_enemy/st_fire.hpp"
#include "states/attack_enemy/st_mission_accomplished.hpp"
