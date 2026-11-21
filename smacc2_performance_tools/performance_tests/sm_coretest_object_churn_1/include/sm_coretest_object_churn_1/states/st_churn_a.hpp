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

namespace sm_coretest_object_churn_1
{
// One of the two churn states; see the state machine header for what each state holds.
struct StChurnA : smacc2::SmaccState<StChurnA, SmCoretestObjectChurn1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvAllGo<SrAllEventsGo, StChurnA>, StChurnB>> reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTick, CbTickSubscriber>();
    configure_orthogonal<OrAsync, CbFinish>();
    static_createEventGenerator<EgConditionalGenerator>(ConditionalGeneratorMode::ON_UPDATE);
    static_createStateReactor<
      SrAllEventsGo,
      // Set Reactor Output Event
      EvAllGo<SrAllEventsGo, StChurnA>,
      // Add Reactor Input Events
      mpl::list<EvCbSuccess<CbFinish, OrAsync>, EvTrue<EgConditionalGenerator, StChurnA>>>();
  }

  void runtimeConfigure()
  {
    this->getEventGenerator<EgConditionalGenerator>()->setPredicateFunction([] { return true; });
  }

  void onEntry()
  {
    this->requiresComponent(tick_);
    tick_->onTimerTick(&StChurnA::onTick, this);
    auto & sm = this->context<SmCoretestObjectChurn1>();
    if (++sm.roundTrips_ % 1000 == 0)
    {
      // FATAL so that the count is printed with logging otherwise off
      RCLCPP_FATAL(getLogger(), "[churn] round trips: %lu", sm.roundTrips_);
    }
  }

  void onTick() { ++ticks_; }

  smacc2::client_core_components::CpRos2Timer * tick_ = nullptr;
  unsigned long ticks_ = 0;
};
}  // namespace sm_coretest_object_churn_1
