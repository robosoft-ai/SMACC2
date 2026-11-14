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
// A sequence of ten sleeps against a 330 ms timer: the state always exits with the sequence part
// way through, between two of the sequence's own 200 ms polls.
struct StSequenceCut : smacc2::SmaccState<StSequenceCut, SmCoretestFeatures1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, StSequenceCutOnStep>,
    Transition<EvCbSuccess<CbSequence, OrWork>, StTrap>,
    Transition<EvCbFailure<CbSequence, OrWork>, StTrap>>
    reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrWork, CbSequence>();
    configure_orthogonal<OrTimer, CbTimerCountdownLoop>(330ms);
  }

  void runtimeConfigure()
  {
    auto * sequence = this->getClientBehavior<OrWork, CbSequence>();
    for (int i = 0; i < 10; ++i) sequence->then<OrWork, CbSleepFor>(rclcpp::Duration(50ms));
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StSequenceCut] Entered."); }
};
}  // namespace sm_coretest_features_1
