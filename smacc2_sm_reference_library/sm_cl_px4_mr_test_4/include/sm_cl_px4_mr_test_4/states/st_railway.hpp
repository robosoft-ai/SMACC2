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

#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_plan.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace smacc2::default_transition_tags;

// STATE: central dispatcher ("railway"). Every leg state and every pattern
// superstate returns here; on each visit the railway advances the plan cursor
// and posts EvRailwayDispatch<Target> for the next leg. One transition row per
// reachable target keeps every superstate independently reachable.
//
// No behaviors run here: CpOffboardKeepAlive keeps republishing the last
// setpoint, and the railway is resident for a single scheduler turn.
struct StRailway : smacc2::SmaccState<StRailway, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    // transit nav states
    Transition<EvRailwayDispatch<StSineWaveVertical>, StSineWaveVertical, DISPATCH>,
    Transition<EvRailwayDispatch<StSineWaveHorizontal>, StSineWaveHorizontal, DISPATCH>,
    Transition<EvRailwayDispatch<StGoToWaypoint>, StGoToWaypoint, DISPATCH>,
    // tail pattern states
    Transition<EvRailwayDispatch<StFigureEight1>, StFigureEight1, DISPATCH>,
    Transition<EvRailwayDispatch<StFigureEight2>, StFigureEight2, DISPATCH>,
    Transition<EvRailwayDispatch<StLoiterCentroid>, StLoiterCentroid, DISPATCH>,
    // pattern superstates
    Transition<EvRailwayDispatch<SsSquareSpiral1>, SsSquareSpiral1, DISPATCH>,
    Transition<EvRailwayDispatch<SsSquareSpiral2>, SsSquareSpiral2, DISPATCH>,
    Transition<EvRailwayDispatch<SsLawnmower1>, SsLawnmower1, DISPATCH>,
    Transition<EvRailwayDispatch<SsLawnmower2>, SsLawnmower2, DISPATCH>,
    Transition<EvRailwayDispatch<SsGridPattern1>, SsGridPattern1, DISPATCH>,
    Transition<EvRailwayDispatch<SsVSSearch1>, SsVSSearch1, DISPATCH>,
    Transition<EvRailwayDispatch<SsVSSearch2>, SsVSSearch2, DISPATCH>,
    Transition<EvRailwayDispatch<SsVSSearch3>, SsVSSearch3, DISPATCH>,
    Transition<EvRailwayDispatch<SsVSChain1>, SsVSChain1, DISPATCH>,
    Transition<EvRailwayDispatch<SsVSChain2>, SsVSChain2, DISPATCH>,
    Transition<EvRailwayDispatch<SsVSChain3>, SsVSChain3, DISPATCH>,
    // landing chain
    Transition<EvRailwayDispatch<StGoToLandingZone>, StGoToLandingZone, DISPATCH>
  > reactions;

  static void staticConfigure() {}

  void runtimeConfigure()
  {
    auto & plan = this->context<MsInFlight>().plan;

    // advance past the leg that just returned to us
    if (plan.legInProgress)
    {
      plan.legInProgress = false;
      plan.cursor++;
    }

    if (!plan.valid)
    {
      RCLCPP_ERROR(getLogger(), "StRailway: plan invalid - dispatching landing");
      this->postEvent<EvRailwayDispatch<StGoToLandingZone>>();
      return;
    }

    if (plan.exhausted())
    {
      RCLCPP_INFO(getLogger(), "StRailway: all legs consumed - dispatching landing");
      this->postEvent<EvRailwayDispatch<StGoToLandingZone>>();
      return;
    }

    const auto & leg = plan.current();
    plan.legInProgress = true;
    RCLCPP_INFO(
      getLogger(), "StRailway: dispatching leg L%d %s %s (%s, %.0f m)", leg.index,
      railway::legKindName(leg.kind),
      leg.kind == railway::LegKind::PATTERN ? railway::roleName(leg.role) : "", leg.label.c_str(),
      static_cast<double>(leg.lengthM));

    switch (leg.kind)
    {
      case railway::LegKind::TRANSIT_V:
        this->postEvent<EvRailwayDispatch<StSineWaveVertical>>();
        break;
      case railway::LegKind::TRANSIT_H:
        this->postEvent<EvRailwayDispatch<StSineWaveHorizontal>>();
        break;
      case railway::LegKind::GOTO:
        this->postEvent<EvRailwayDispatch<StGoToWaypoint>>();
        break;
      case railway::LegKind::LANDING:
        this->postEvent<EvRailwayDispatch<StGoToLandingZone>>();
        break;
      case railway::LegKind::PATTERN:
        dispatchPattern(leg.role);
        break;
      default:
        RCLCPP_ERROR(getLogger(), "StRailway: unknown leg kind - dispatching landing");
        this->postEvent<EvRailwayDispatch<StGoToLandingZone>>();
        break;
    }
  }

  void onEntry() {}
  void onExit() {}

private:
  void dispatchPattern(railway::NodeRole role)
  {
    switch (role)
    {
      case railway::NodeRole::PIN_SQUARE_SPIRAL_1:
        this->postEvent<EvRailwayDispatch<SsSquareSpiral1>>();
        break;
      case railway::NodeRole::PIN_SQUARE_SPIRAL_2:
        this->postEvent<EvRailwayDispatch<SsSquareSpiral2>>();
        break;
      case railway::NodeRole::PIN_LAWNMOWER_1:
        this->postEvent<EvRailwayDispatch<SsLawnmower1>>();
        break;
      case railway::NodeRole::PIN_LAWNMOWER_2:
        this->postEvent<EvRailwayDispatch<SsLawnmower2>>();
        break;
      case railway::NodeRole::PIN_GRID_PATTERN_1:
        this->postEvent<EvRailwayDispatch<SsGridPattern1>>();
        break;
      case railway::NodeRole::PIN_VS_SEARCH_1:
        this->postEvent<EvRailwayDispatch<SsVSSearch1>>();
        break;
      case railway::NodeRole::PIN_VS_SEARCH_2:
        this->postEvent<EvRailwayDispatch<SsVSSearch2>>();
        break;
      case railway::NodeRole::PIN_VS_SEARCH_3:
        this->postEvent<EvRailwayDispatch<SsVSSearch3>>();
        break;
      case railway::NodeRole::PIN_VS_CHAIN_1:
        this->postEvent<EvRailwayDispatch<SsVSChain1>>();
        break;
      case railway::NodeRole::PIN_VS_CHAIN_2:
        this->postEvent<EvRailwayDispatch<SsVSChain2>>();
        break;
      case railway::NodeRole::PIN_VS_CHAIN_3:
        this->postEvent<EvRailwayDispatch<SsVSChain3>>();
        break;
      case railway::NodeRole::FIGURE_EIGHT_1:
        this->postEvent<EvRailwayDispatch<StFigureEight1>>();
        break;
      case railway::NodeRole::FIGURE_EIGHT_2:
        this->postEvent<EvRailwayDispatch<StFigureEight2>>();
        break;
      case railway::NodeRole::LOITER_CENTROID:
        this->postEvent<EvRailwayDispatch<StLoiterCentroid>>();
        break;
      default:
        RCLCPP_ERROR(getLogger(), "StRailway: no superstate for role %s - landing", railway::roleName(role));
        this->postEvent<EvRailwayDispatch<StGoToLandingZone>>();
        break;
    }
  }
};

}  // namespace sm_cl_px4_mr_test_4
