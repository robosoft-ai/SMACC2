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
#include "sm_panda_moveit2z_cb_inventory/orthogonals/or_arm.hpp"
#include "sm_panda_moveit2z_cb_inventory/orthogonals/or_keyboard.hpp"

#include <moveit2z_client/cl_moveit2z.hpp>
#include <moveit2z_client/client_behaviors.hpp>

#include <keyboard_client/cl_keyboard.hpp>
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>

using namespace cl_moveit2z;
using namespace cl_keyboard;


namespace sm_panda_moveit2z_cb_inventory
{

//STATES
struct StAcquireSensors;
struct StPause1;
struct StPause2;
struct StPause3;
struct StPause4;
struct StPause5;
struct StPause6;
struct StMoveJoints1;
struct StMoveJoints2;
struct StMoveJoints3;
struct StMoveEndEffector;
struct StMoveCartesianRelative;
struct StMoveCartesianRelative2;
struct StCircularPivotMotion;
struct StAttachObject;
struct StDetatchObject;
struct StEndEffectorRotate;
struct StExecuteLastTrajectory;
struct StMoveKnownState1;
struct StMoveKnownState2;
struct StMoveKnownState3;
struct StPouringMotion;
struct StUndoLastTrajectory;

//--------------------------------------------------------------------
//STATE_MACHINE

struct SmPandaMoveit2zCbInventory : public smacc2::SmaccStateMachineBase<SmPandaMoveit2zCbInventory, StPause1>

{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override { 
    this->createOrthogonal<OrArm>(); 
    this->createOrthogonal<OrKeyboard>();
    }
};

}  // namespace sm_panda_moveit2z_cb_inventory

// STATES
#include "states/st_acquire_sensors.hpp"
#include "states/st_pause_1.hpp"
#include "states/st_pause_2.hpp"
#include "states/st_pause_3.hpp"
#include "states/st_pause_4.hpp"
#include "states/st_pause_5.hpp"
#include "states/st_pause_6.hpp"
#include "states/st_attach_object.hpp"
#include "states/st_move_end_effector.hpp"
#include "states/st_circular_pivot_motion.hpp"
#include "states/st_move_joints_1.hpp"
#include "states/st_move_joints_2.hpp"
#include "states/st_move_joints_3.hpp"
#include "states/st_detach_object.hpp"
#include "states/st_move_known_state_1.hpp"
#include "states/st_move_known_state_2.hpp"
#include "states/st_move_known_state_3.hpp"
#include "states/st_end_effector_rotate.hpp"
#include "states/st_move_last_trajectory_initial_state.hpp"
#include "states/st_execute_last_trajectory.hpp"
#include "states/st_move_named_target.hpp"
#include "states/st_move_cartesian_relative2.hpp"
#include "states/st_pouring_motion.hpp"
#include "states/st_move_cartesian_relative.hpp"
#include "states/st_undo_last_trajectory.hpp"
