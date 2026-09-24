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

#include <cl_px4_mr/client_behaviors/cb_figure_eight.hpp>
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/states/st_go_to_south_waypoint.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// TAIL PATTERN STATE: figure-eight loiter (CbFigureEight, parametric
// lemniscate), lobes north-south. Owns the centroid that StFigureEight2 and
// StLoiterCentroid share, and the lobe-tip entry StGoToFigureEight flies to.
struct StFigureEight1 : smacc2::SmaccState<StFigureEight1, MsInFlight>
{
  using SmaccState::SmaccState;

  // the centroid: kFigureEightOffsetSouthM south of the SouthWaypoint
  static NedXY centre()
  {
    NedXY c = StGoToSouthWaypoint::target();
    c.x -= kFigureEightOffsetSouthM;
    return c;
  }

  static constexpr float heading() { return kFigureEightHeading1; }  // lobe axis, NED yaw

  // CbFigureEight starts (and ends) at the lobe tip along its axis
  static NedXY entry()
  {
    NedXY e = centre();
    e.x += kFigureEightHalfLengthM * std::cos(heading());
    e.y += kFigureEightHalfLengthM * std::sin(heading());
    return e;
  }

  typedef mpl::list<
    Transition<EvCbSuccess<CbFigureEight, OrPx4>, StFigureEight2, SUCCESS>,
    Transition<EvCbFailure<CbFigureEight, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    const NedXY c = centre();
    configure_orthogonal<OrPx4, CbFigureEight>(
      c.x, c.y, kMissionAltitudeM, kFigureEightHalfLengthM,
      kFigureEightSpeedRad, kFigureEightLoops);
  }

  void runtimeConfigure()
  {
    auto * cb = this->getClientBehavior<OrPx4, CbFigureEight>();
    cb->setHeading(heading());

    const float loopSeconds = 2.0f * kPi / kFigureEightSpeedRad;
    cb->setTimeout(std::chrono::seconds(static_cast<long>(
      loopSeconds * kFigureEightLoops * kTimeoutMarginFactor + kTimeoutBaseS)));

    const NedXY c = centre();
    RCLCPP_INFO(
      getLogger(), "StFigureEight1: centre NED (%.1f, %.1f), axis %.0f deg, %d loop(s), ~%.0f s",
      static_cast<double>(c.x), static_cast<double>(c.y), heading() * 180.0 / M_PI,
      kFigureEightLoops, static_cast<double>(loopSeconds * kFigureEightLoops));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
