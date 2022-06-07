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

// ORTHOGONALS
#include "orthogonals/or_arm_left.hpp"
#include "orthogonals/or_arm_right.hpp"

// CLIENTS
#include <move_group_interface_client/cl_movegroup.hpp>
#include <move_group_interface_client/client_behaviors.hpp>

// CLIENT BEHAVIORS
#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>
#include "clients/move_group/client_behaviors/cb_wait_joint_states.hpp"

namespace sm_multi_panda_sim
{

using namespace cl_move_group_interface;

//STATES
struct StAcquireSensors;
struct StMoveJoints;


//--------------------------------------------------------------------
//STATE_MACHINE
struct SmMultiPandaSim : public smacc2::SmaccStateMachineBase<SmMultiPandaSim, StAcquireSensors>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override 
  { 
    this->createOrthogonal<OrArmLeft>(); 
    this->createOrthogonal<OrArmRight>(); 
    }
};

}  // namespace sm_multi_panda_sim

// STATES
#include "states/st_acquire_sensors.hpp"
#include "states/st_move_joints.hpp"
