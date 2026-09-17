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

#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: offboard takeoff to the initial altitude
struct StTakeoff : smacc2::SmaccState<StTakeoff, MsTakeoff>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbTakeOff, OrPx4>, MsInFlight, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbTakeOff>(railway::kTakeoffAltitudeM);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StTakeoff: taking off to %.1f m", railway::kTakeoffAltitudeM);
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StTakeoff: takeoff complete");
  }
};

}  // namespace sm_cl_px4_mr_test_4
