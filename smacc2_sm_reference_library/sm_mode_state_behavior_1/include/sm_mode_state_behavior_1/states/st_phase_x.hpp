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
struct StPhaseX : smacc2::SmaccState<StPhaseX, SsPhase2>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StPhaseY>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StPhaseY>
    > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5s);
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StPhaseX] Active. Press 'n' or wait 5s → PhaseY."); }
};
}  // namespace sm_mode_state_behavior_1
