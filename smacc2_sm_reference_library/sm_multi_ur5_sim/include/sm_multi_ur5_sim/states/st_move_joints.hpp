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

#define UR5_PREFIX "ur5_2"
namespace sm_multi_ur5_sim
{
// SMACC2 classes
using smacc2::EvStateRequestFinish;
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using namespace smacc2;
using namespace cl_move_group_interface;


// STATE DECLARATION
struct StMoveJoints : smacc2::SmaccState<StMoveJoints, SmTestMoveitUr5Sim>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
    Transition<EvCbSuccess<CbMoveJoints, OrArm>, StMoveEndEffector, SUCCESS>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    std::map<std::string, double> jointValues{
      {UR5_PREFIX"shoulder_lift_joint", 0.0},
      {UR5_PREFIX"shoulder_pan_joint", 0.0},
      {UR5_PREFIX"wrist_1_joint", M_PI / 4},
      {UR5_PREFIX"wrist_2_joint", 0.0},
      {UR5_PREFIX"wrist_3_joint", 0.0}};

    configure_orthogonal<OrArm, CbMoveJoints>(jointValues);
  };

  void runtimeConfigure()
  {
    ClMoveGroup * moveGroupClient;
    this->requiresClient(moveGroupClient);
    this->getClientBehavior<OrArm,CbMoveJoints>()->scalingFactor_ = 1;
  }
};
}  // namespace sm_multi_ur5_sim
