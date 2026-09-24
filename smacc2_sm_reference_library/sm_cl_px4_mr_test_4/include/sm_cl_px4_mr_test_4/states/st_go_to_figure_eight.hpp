// Copyright 2026 RobosoftAI Inc.
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

#pragma once

#include <smacc2/smacc.hpp>

#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/states/st_go_to_south_waypoint.hpp>
#include <sm_cl_px4_mr_test_4/states/st_figure_eight_1.hpp>


#include <algorithm>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// NAV STATE: straight leg to the lobe tip where figure-eight 1 starts
struct StGoToFigureEight : smacc2::SmaccState<StGoToFigureEight, MsInFlight>
{
  using SmaccState::SmaccState;

  // the lobe tip where figure-eight 1 starts
  static NedXY target()
  {
    return StFigureEight1::entry();
  }

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, StFigureEight1, SUCCESS>,
    Transition<EvCbFailure<CbGoToLocation, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    const NedXY to = target();
    configure_orthogonal<OrPx4, CbGoToLocation>(to.x, to.y, -kMissionAltitudeM);
  }

  void runtimeConfigure()
  {
    const NedXY from = StGoToSouthWaypoint::target();
    const NedXY to = target();
    const float lengthM = distanceM(from, to);
    this->getClientBehavior<OrPx4, CbGoToLocation>()->setTimeout(
      transitTimeout(std::max(lengthM, 50.0f)));

    RCLCPP_INFO(
      getLogger(), "StGoToFigureEight: -> FigureEight lobe tip NED (%.1f, %.1f), %.0f m", static_cast<double>(to.x),
      static_cast<double>(to.y), static_cast<double>(lengthM));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
