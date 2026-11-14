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
// The same sequence against a 200 ms timer, which is the sequence's own polling period: the state
// exits just as the sequence starts its second step, the moment that used to deadlock the machine.
struct StSequenceCutOnStep : smacc2::SmaccState<StSequenceCutOnStep, SmCoretestFeatures1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StGenerator>,
    Transition<EvCbSuccess<CbSequence, OrWork>, StTrap>,
    Transition<EvCbFailure<CbSequence, OrWork>, StTrap>>
    reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrWork, CbSequence>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(200ms);
  }

  void runtimeConfigure()
  {
    auto * sequence = this->getClientBehavior<OrWork, CbSequence>();
    for (int i = 0; i < 10; ++i) sequence->then<OrWork, CbSleepFor>(rclcpp::Duration(50ms));
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StSequenceCutOnStep] Entered."); }
};
}  // namespace sm_coretest_features_1
