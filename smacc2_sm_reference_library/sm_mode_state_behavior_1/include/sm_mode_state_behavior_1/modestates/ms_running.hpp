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
// MsRunning: Mode state that owns the SINGLE CbDefaultKeyboardBehavior for the
// entire machine. Because keyboard behavior lives only here, every keypress
// generates exactly ONE event regardless of which inner leaf state is active.
// Leaf states react to EvKeyPressN without having their own keyboard behavior,
// demonstrating that the event source and the reacting state can be at different
// hierarchy levels.
class MsRunning : public smacc2::SmaccState<MsRunning, SmModeStateBehavior1, SsPhase1>
{
public:
  using SmaccState::SmaccState;

  // Keyboard behavior is declared here at the mode state level.
  // It persists across all SsPhase1 <-> SsPhase2 switches and across
  // all leaf state transitions within each superstate.
  static void staticConfigure()
  {
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[MsRunning] Entered. Keyboard behavior active."); }

  void onExit() { RCLCPP_INFO(getLogger(), "[MsRunning] Exiting."); }
};
}  // namespace sm_mode_state_behavior_1
