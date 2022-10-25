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
#include <nav2z_client/nav2z_client.hpp>
#include <nav2z_client/client_behaviors.hpp>
#include <sm_husky_barrel_search_1/clients/cb_sleep_for.hpp>
#include <sm_husky_barrel_search_1/clients/led_array/client_behaviors.hpp>
#include <smacc2/client_behaviors/cb_sequence.hpp>
#include <nav2z_client/components/waypoints_navigator/waypoints_navigator.hpp>

namespace sm_husky_barrel_search_1
{
using cl_nav2z::CbNavigateBackwards;
using cl_nav2z::CbAbsoluteRotate;
using cl_nav2z::CbRotateLookAt;
using cl_nav2z::WaypointNavigator;

using sm_husky_barrel_search_1::cl_led_array::CbBlinking;
using smacc2::client_behaviors::CbSequence;

// STATE DECLARATION
struct StBackupFromParking : smacc2::SmaccState<StBackupFromParking, SmHuskyBarrelSearch1>
{
  using SmaccState::SmaccState;

  // // TRANSITION TABLE
  typedef mpl::list<
                    Transition<EvCbSuccess<CbSequence, OrNavigation>, StMoveBaseEntrance>,
                    //Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StExitBase>,
                    Transition<EvCbFailure<CbSequence, OrNavigation>, StBackupFromParking>
                    >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbSequence>();
    configure_orthogonal<OrLedArray, CbBlinking>(LedColor::YELLOW);
    configure_orthogonal<OrLedArray, CbLEDOff>(LedColor::RED);
    configure_orthogonal<OrLedArray, CbLEDOff>(LedColor::GREEN);
  }

  void runtimeConfigure()
  {
    // cl_nav2z::ClNav2Z * nav2zClient;
    // requiresClient(nav2zClient);

    // auto cpWaypointsNavigator = nav2zClient->getComponent<WaypointNavigator>();
    // auto currentPoseStamped = nav2zClient->getComponent<cl_nav2z::Pose>()->toPoseStampedMsg();

    // auto lookat_pose = cpWaypointsNavigator->getNamedPose("military-pickup-green");
    // geometry_msgs::msg::PoseStamped pose_stamped = currentPoseStamped;
    // pose_stamped.pose = *lookat_pose;

    auto cbsequence = this->template getClientBehavior<OrNavigation, CbSequence>();
    cbsequence
      ->then<OrNavigation, CbNavigateBackwards>(6.0)
      // ->then<OrNavigation, CbSleepFor>(1s)
      // ->then<OrNavigation, CbRotateLookAt>(pose_stamped)
      // ->then<OrNavigation, CbSleepFor>(1s)
      ->then<OrNavigation, CbAbsoluteRotate>(-90.0);
  }
};
}  // namespace sm_husky_barrel_search_1
