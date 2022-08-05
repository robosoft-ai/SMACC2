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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc2/smacc.hpp>
namespace sm_husky_barrel_search_1
{
// STATE DECLARATION
struct StFire : smacc2::SmaccState<StFire, SmHuskyBarrelSearch1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
                    Transition<EvCbSuccess<CbSequence, OrNavigation>, StMissionAccomplished>,
                    //Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StExitBase>,
                    Transition<EvCbFailure<CbSequence, OrNavigation>, StFire>
                    >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbSequence>();
    configure_orthogonal<OrLedArray, CbBlinking>(LedColor::YELLOW);
  }

  WaypointNavigator* cpWaypointsNavigator;
  cl_nav2z::Pose* currentPose;

  geometry_msgs::msg::PoseStamped getLookAtPosition(std::string enemyName)
  {
    auto currentPoseStamped = currentPose->toPoseStampedMsg();
    auto lookat_pose = cpWaypointsNavigator->getNamedPose(enemyName);
    geometry_msgs::msg::PoseStamped pose_stamped = currentPoseStamped;
    pose_stamped.pose = *lookat_pose;
    return pose_stamped;
  }

  void runtimeConfigure()
  {
    cl_nav2z::ClNav2Z* moveBaseClient;

    requiresClient(moveBaseClient);

    cpWaypointsNavigator = moveBaseClient->getComponent<WaypointNavigator>();
    currentPose = moveBaseClient->getComponent<cl_nav2z::Pose>();
    
    auto enemyPosition1 = getLookAtPosition("enemy-military-pickup-1");
    auto enemyPosition2 = getLookAtPosition("enemy-military-pickup-2");
    auto enemyPosition3 = getLookAtPosition("enemy-military-pickup-3");


    auto cbsequence = this->template getClientBehavior<OrNavigation, CbSequence>();

    cbsequence
        ->then<OrNavigation, CbRotateLookAt>(enemyPosition1)
        ->then<OrNavigation, CbSleepFor>(3s)
        ->then<OrNavigation, CbRotateLookAt>(enemyPosition2)
        ->then<OrNavigation, CbSleepFor>(3s)
        ->then<OrNavigation, CbRotateLookAt>(enemyPosition3);
  }
};
}  // namespace sm_husky_barrel_search_1
