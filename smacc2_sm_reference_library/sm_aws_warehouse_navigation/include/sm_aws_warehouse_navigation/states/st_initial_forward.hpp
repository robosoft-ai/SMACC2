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

#pragma once
#include <smacc2/smacc.hpp>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>


namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StInitialNavigateForward : smacc2::SmaccState<StInitialNavigateForward, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

  Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateMainAisle>,
  Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StInitialNavigateForward, ABORT>

  >
  reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrNavigation, CbNavigateForward>(2);

  }


  void runtimeConfigure() {}

  void onEntry()
  {
  }

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
