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
#include "sm_test_moveit_ur5_sim/orthogonals/or_arm.hpp"

#include <move_group_interface_client/cl_movegroup.hpp>
#include <move_group_interface_client/client_behaviors.hpp>

namespace sm_test_moveit_ur5_sim
{

using namespace cl_move_group_interface;

//STATES
struct StMoveJoints;
struct StMoveCartesian;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmTestMoveitUr5Sim : public smacc2::SmaccStateMachineBase<SmTestMoveitUr5Sim, StMoveJoints>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override { this->createOrthogonal<OrArm>(); }
};

}  // namespace sm_test_moveit_ur5_sim

// STATES
#include "states/st_move_joints.hpp"
#include "states/st_move_cartesian.hpp"
#include "states/st_move_cartesian_relative.hpp"
