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

namespace sm_dance_bot_warehouse_2
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialEndPoint : smacc2::SmaccState<StiRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiRadialReturn, SUCCESS>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiRadialReturn, ABORT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //RCLCPP_INFO(getLogger(),"ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
    // float EXTRA_SECURE_DISTANCE = 0.5;

    ClNav2Z* navzclient;
    this->requiresClient(navzclient);

    //auto lidarData = lidarClient->getComponent<CpForwardObstacleDetector>();
    auto forwardDistanceComputation = navzclient->getComponent<CpSquareShapeBoundary>();

    auto forwardBehavior =
      this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateForward>();

    //forwardBehavior->setForwardDistance(std::min(forwardDistanceComputation->getForwardDistance() + EXTRA_SECURE_DISTANCE, 6.0f)); // at most 15 meters
    forwardBehavior->setForwardDistance(forwardDistanceComputation->getForwardDistance() ); // at most 15 meters
  }
};
}  // namespace radial_motion_states
}  // namespace sm_dance_bot_warehouse_2
