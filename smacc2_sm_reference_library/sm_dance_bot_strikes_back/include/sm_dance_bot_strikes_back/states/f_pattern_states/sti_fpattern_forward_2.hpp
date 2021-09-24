// Copyright 2021 RobosoftAI Inc.
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

namespace sm_dance_bot_strikes_back
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternForward2 : smacc2::SmaccState<StiFPatternForward2<SS>, SS>
{
  typedef SmaccState<StiFPatternForward2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternRotate2<SS>>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure()
  {
    // auto &superstate = TSti::template context<SS>();

    TSti::template configure<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    TSti::template configure<OrLED, CbLEDOff>();
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot_strikes_back
