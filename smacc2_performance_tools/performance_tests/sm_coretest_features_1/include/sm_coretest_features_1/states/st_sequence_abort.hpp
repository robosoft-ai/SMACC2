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
// The second step of the sequence posts its failure event: EvCbFailure of the sequence, and the
// third step never runs.
struct StSequenceAbort : smacc2::SmaccState<StSequenceAbort, SmCoretestFeatures1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbFailure<CbSequence, OrWork>, StPeriodic>,
    Transition<EvCbSuccess<CbSequence, OrWork>, StTrap>>
    reactions;

  static void staticConfigure() { configure_orthogonal<OrWork, CbSequence>(); }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrWork, CbSequence>()
      ->then<OrWork, CbSleepFor>(rclcpp::Duration(50ms))
      ->then<OrWork, CbFail>()
      ->then<OrWork, CbSleepFor>(rclcpp::Duration(50ms));
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StSequenceAbort] Entered."); }
};
}  // namespace sm_coretest_features_1
