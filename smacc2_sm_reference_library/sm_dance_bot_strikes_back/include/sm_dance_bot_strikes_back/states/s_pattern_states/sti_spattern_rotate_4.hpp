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

namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternRotate4 : smacc2::SmaccState<StiSPatternRotate4, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiSPatternForward4>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiSPatternRotate4>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();
    RCLCPP_INFO(
      getLogger(), "[SsrSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d",
      superstate.iteration_count, SS::total_iterations());

    float offset = 0;
    float angle = 0;
    if (superstate.direction() == TDirection::LEFT)
      angle = -90 - offset;
    else
      angle = 90 + offset;

    this->configure<OrNavigation, CbAbsoluteRotate>(angle);
    this->configure<OrNavigation, CbResumeSlam>();
    this->configure<OrLED, CbLEDOff>();
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot_strikes_back
