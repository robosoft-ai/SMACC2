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

namespace sm_cl_px4_mr_test_4
{

using namespace smacc2::default_transition_tags;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;

// STATE: let PX4 topics settle before loading the mission
struct StWaitForReady : smacc2::SmaccState<StWaitForReady, MsDisarmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrPx4>, StLoadMission, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSleepFor>(rclcpp::Duration(5s));
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StWaitForReady: waiting 5 s for PX4 topics...");
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
