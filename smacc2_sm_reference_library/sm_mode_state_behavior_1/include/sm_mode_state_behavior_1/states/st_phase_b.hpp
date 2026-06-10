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

namespace sm_mode_state_behavior_1
{
struct StPhaseB : smacc2::SmaccState<StPhaseB, SsPhase1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StPhaseC>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StPhaseC>
    > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(3s);
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StPhaseB] Active. Press 'n' or wait 3s → PhaseC."); }
};
}  // namespace sm_mode_state_behavior_1
