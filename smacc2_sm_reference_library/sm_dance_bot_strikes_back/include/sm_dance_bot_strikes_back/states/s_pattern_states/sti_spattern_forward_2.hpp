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

namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward2 : public smacc2::SmaccState<StiSPatternForward2, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiSPatternRotate3>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiSPatternRotate2>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>();
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();

    double extrasecurityMargin = 0.2;

    auto forwardBehavior =
      this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateForward>();

    cl_lidar::ClLidarSensor * lidarClient;
    this->requiresClient(lidarClient);
    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    if (!std::isnan(lidarData->forwardObstacleDistance))
      forwardBehavior->forwardDistance =
        lidarData->forwardObstacleDistance -
        extrasecurityMargin; /*extra security margin for easy dynamic implementation of dynamic-smotion*/

    else
      forwardBehavior->forwardDistance = superstate.pitch2_lenght_meters();
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot_strikes_back
