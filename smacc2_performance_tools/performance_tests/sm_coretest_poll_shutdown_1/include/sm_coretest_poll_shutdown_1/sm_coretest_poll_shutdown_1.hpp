// Copyright 2025 Robosoft Inc.
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

#include <sm_coretest_poll_shutdown_1/orthogonals/or_slow.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace std::chrono_literals;

namespace sm_coretest_poll_shutdown_1
{
struct StIdle;

struct SmCoretestPollShutdown1
: public smacc2::SmaccStateMachineBase<SmCoretestPollShutdown1, StIdle>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override { this->createOrthogonal<OrSlow>(); }
};
}  // namespace sm_coretest_poll_shutdown_1

#include <sm_coretest_poll_shutdown_1/states/st_idle.hpp>
