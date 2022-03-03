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
namespace f_pattern_states
{
  using sm_dance_bot_warehouse_2::cl_lidar::CpForwardObstacleDetector;

// STATE DECLARATION
template <typename SS>
struct StiFPatternForward1 : public smacc2::SmaccState<StiFPatternForward1<SS>, SS>
{
  typedef SmaccState<StiFPatternForward1<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>, ABORT>


    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>();
    TSti::template configure_orthogonal<OrNavigation, CbPauseSlam>();
    TSti::template configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
      cl_lidar::ClLidarSensor * lidarClient;
      this->requiresClient(lidarClient);

      auto lidarData = lidarClient->getComponent<CpForwardObstacleDetector>();

      auto forwardBehavior =
        TSti::template getOrthogonal<OrNavigation>()->template getClientBehavior<CbNavigateForward>();

      forwardBehavior->setForwardDistance( lidarData->getForwardDistance());
      RCLCPP_INFO(
        this->getLogger(), "Going forward in F pattern, distance to wall: %lf",
        lidarData->getForwardDistance());
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot_warehouse_2
