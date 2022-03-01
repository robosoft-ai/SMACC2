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
