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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"
#include <boost/mpl/list.hpp>

// CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

// #include <sm_pack_ml/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.hpp>
// #include <sm_pack_ml/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.hpp>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>

// STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

// ORTHOGONALS
#include "orthogonals/or_arm_left.hpp"
#include "orthogonals/or_arm_right.hpp"
#include "orthogonals/or_timer.hpp"
#include "orthogonals/or_keyboard.hpp"
#include "orthogonals/or_updatable_publisher.hpp"

// CLIENTS
#include <moveit2z_client/cl_moveit2z.hpp>
#include <moveit2z_client/client_behaviors.hpp>

// CLIENT BEHAVIORS
#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>
#include "clients/move_group/client_behaviors/cb_wait_joint_states.hpp"
#include "clients/move_group/client_behaviors/cb_sleep_for.hpp"
#include "clients/move_group/client_behaviors/cb_move_synchronized_link_goals.hpp"

namespace sm_multi_panda_sim
{

using namespace cl_moveit2z;
using namespace cl_ros_timer;
using namespace cl_keyboard;
using namespace boost;
using namespace smacc2;

using namespace smacc2::default_events;

// STATES
struct StAcquireSensors;
class StLeftArmMoves;
class StRightArmMoves;
class StBothArmsMove;

struct EvToDeep : sc::event<EvToDeep>{};
struct EvFail : sc::event<EvFail>{};
struct EvSc : sc::event<EvSc>{};
struct EvStart : sc::event<EvStart>{};
struct EvReset : sc::event<EvReset>{};
struct EvSuspend : sc::event<EvSuspend>{};
struct EvUnSuspend : sc::event<EvUnSuspend>{};
struct EvHold : sc::event<EvHold>{};
struct EvUnhold : sc::event<EvUnhold>{};
struct EvStop : sc::event<EvStop>{};
struct EvClear : sc::event<EvClear>{};

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmMultiPandaSim : public smacc2::SmaccStateMachineBase<SmMultiPandaSim, StAcquireSensors>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrArmLeft>();
    this->createOrthogonal<OrArmRight>();
    this->createOrthogonal<OrKeyboard>();
  }
};

}  // namespace sm_multi_panda_sim

// STATES
#include "states/st_acquire_sensors.hpp"

#include "states/st_left_arm_moves.hpp"
#include "states/st_right_arm_moves.hpp"
#include "states/st_both_arms_move.hpp"
