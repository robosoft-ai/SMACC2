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

namespace sm_coretest_features_1
{
// An event generator in update mode: it posts EvFalse on every update until the state has seen six
// updates, then EvTrue, which is the only event the state reacts to.
struct StGenerator : smacc2::SmaccState<StGenerator, SmCoretestFeatures1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTrue<EgConditionalGenerator, StGenerator>, StSequence>> reactions;

  static void staticConfigure()
  {
    static_createEventGenerator<EgConditionalGenerator>(ConditionalGeneratorMode::ON_UPDATE);
  }

  void runtimeConfigure()
  {
    updates_ = 0;
    this->getEventGenerator<EgConditionalGenerator>()->setPredicateFunction(
      [this] { return ++updates_ > 6; });
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StGenerator] Entered."); }

  int updates_ = 0;
};
}  // namespace sm_coretest_features_1
