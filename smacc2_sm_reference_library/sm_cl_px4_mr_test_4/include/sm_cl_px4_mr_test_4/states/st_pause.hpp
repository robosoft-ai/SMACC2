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

#include <smacc2/client_behaviors/cb_sleep_for.hpp>
#include <smacc2/smacc.hpp>

#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace smacc2::default_transition_tags;
using smacc2::client_behaviors::CbSleepFor;

// STATE: initial hold on the ground before anything starts, so the operator
// can position the Gazebo camera and windows
struct StPause : smacc2::SmaccState<StPause, MsDisarmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrPx4>, StConnectMicroROSAgent, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSleepFor>(rclcpp::Duration(railway::kInitialPause));
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StPause: holding %ld s before starting the mission",
      static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(railway::kInitialPause).count()));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StPause: starting");
  }
};

}  // namespace sm_cl_px4_mr_test_4
