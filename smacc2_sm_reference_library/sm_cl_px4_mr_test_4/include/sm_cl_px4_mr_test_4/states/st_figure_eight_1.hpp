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
#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>
#include <sm_cl_px4_mr_test_4/railway/pattern_params.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// TAIL PATTERN STATE: figure-eight loiter (CbFigureEight, parametric
// lemniscate) at the plan's current node, lobes north-south
struct StFigureEight1 : smacc2::SmaccState<StFigureEight1, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFigureEight, OrPx4>, StRailway, NEXT>,
    Transition<EvCbFailure<CbFigureEight, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    // centre overridden from the plan in runtimeConfigure
    const railway::FigureEightSpec s = railway::figureEightSpec(1);
    configure_orthogonal<OrPx4, CbFigureEight>(0.0f, 0.0f, s.altitudeAgl, s.size, s.speed, s.loops);
  }

  void runtimeConfigure()
  {
    const auto & plan = this->context<MsInFlight>().plan;
    const auto & node = plan.currentTargetNode();
    auto * cb = this->getClientBehavior<OrPx4, CbFigureEight>();
    const railway::FigureEightSpec s = railway::figureEightSpec(1);
    cb->setCenter(node.x, node.y);
    cb->setHeading(s.heading);

    const float loopSeconds = 2.0f * static_cast<float>(M_PI) / s.speed;
    cb->setTimeout(std::chrono::seconds(static_cast<long>(
      loopSeconds * s.loops * railway::kTimeoutMarginFactor + railway::kTimeoutBaseS)));

    RCLCPP_INFO(
      getLogger(), "StFigureEight1: centre '%s' NED (%.1f, %.1f), axis %.0f deg, %d loop(s), ~%.0f s",
      node.name.c_str(), static_cast<double>(node.x), static_cast<double>(node.y),
      s.heading * 180.0 / M_PI, s.loops, static_cast<double>(loopSeconds * s.loops));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
