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

#include <angles/angles.h>
namespace sm_dance_bot_strikes_back
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternRotate2 : smacc2::SmaccState<StiFPatternRotate2<SS>, SS>
{
  typedef SmaccState<StiFPatternRotate2<SS>, SS> TSti;
  using TSti::context;
  using TSti::getOrthogonal;

  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE

  typedef mpl::list<
    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiFPatternForward1<SS>>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //TSti::template configure_orthogonal<OrNavigation, CbRotate>(angle);
    TSti::template configure_orthogonal<OrNavigation, CbAbsoluteRotate>();  // absolute horizontal
    TSti::template configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto & superstate = TSti::template context<SS>();

    auto initialStateAngle = 0;         //superstate.initialStateAngle;
    double offset = superstate.offset;  // for a better behaving
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = 90 + offset;
    else
      angle = -90 - offset;

    auto absoluteRotateBehavior =
      TSti::template getOrthogonal<OrNavigation>()->template getClientBehavior<CbAbsoluteRotate>();

    auto targetAngle =
      angles::to_degrees(angles::normalize_angle(angles::from_degrees(initialStateAngle + angle)));

    absoluteRotateBehavior->absoluteGoalAngleDegree = targetAngle;
    RCLCPP_INFO(
      this->getLogger(), "Fpattern, rotate to: %lf",
      *(absoluteRotateBehavior->absoluteGoalAngleDegree));
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot_strikes_back
