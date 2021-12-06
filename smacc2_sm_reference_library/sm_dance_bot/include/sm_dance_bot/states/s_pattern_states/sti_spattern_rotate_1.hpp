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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

namespace sm_dance_bot
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternRotate1 : smacc2::SmaccState<StiSPatternRotate1, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiSPatternForward1>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiSPatternRotate1>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();

    float offset = 0;
    if (superstate.direction() == TDirection::RIGHT)
    {
      // - offset because we are looking to the north and we have to turn clockwise
      this->configure<OrNavigation, CbAbsoluteRotate>(0 - offset);
    }
    else
    {
      // - offset because we are looking to the south and we have to turn counter-clockwise
      this->configure<OrNavigation, CbAbsoluteRotate>(180 + offset);
    }

    this->configure<OrLED, CbLEDOff>();
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot
