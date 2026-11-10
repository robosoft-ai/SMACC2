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
#include <sr_event_countdown/sr_event_countdown.hpp>

#include <sm_coretest_reactor_order_1/events/ev_x.hpp>
#include <sm_coretest_reactor_order_1/orthogonals/or_pulse.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace smacc2::state_reactors;
using namespace std::chrono_literals;

namespace sm_coretest_reactor_order_1
{
struct StA;
struct StViaCountdown;
struct StViaTrigger;

struct SmCoretestReactorOrder1 : public smacc2::SmaccStateMachineBase<SmCoretestReactorOrder1, StA>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override { this->createOrthogonal<OrPulse>(); }
};
}  // namespace sm_coretest_reactor_order_1

#include <sm_coretest_reactor_order_1/states/st_a.hpp>
#include <sm_coretest_reactor_order_1/states/st_via_countdown.hpp>
#include <sm_coretest_reactor_order_1/states/st_via_trigger.hpp>
