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

namespace sm_dance_bot
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward4 : public smacc2::SmaccState<StiSPatternForward4, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiSPatternLoopStart>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiSPatternRotate4>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure()
  {
    //auto &superstate = this->context<SS>();

    this->configure<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    this->configure<OrLED, CbLEDOn>();
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot
