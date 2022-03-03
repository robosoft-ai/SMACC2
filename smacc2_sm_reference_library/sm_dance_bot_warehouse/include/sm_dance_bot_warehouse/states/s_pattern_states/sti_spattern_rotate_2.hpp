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

namespace sm_dance_bot_warehouse
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternRotate2 : smacc2::SmaccState<StiSPatternRotate2, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiSPatternForward2>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiSPatternRotate2>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    float offset = -90;
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = 90 + offset;
    else
      angle = -90 - offset;

    configure_orthogonal<OrNavigation, CbAbsoluteRotate>(angle);
    configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();
    RCLCPP_INFO(
      getLogger(), "[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d",
      superstate.iteration_count, SS::total_iterations());
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot_warehouse
