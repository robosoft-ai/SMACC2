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
struct StiFPatternRotate1 : smacc2::SmaccState<StiFPatternRotate1<SS>, SS>
{
  typedef SmaccState<StiFPatternRotate1<SS>, SS> TSti;
  using TSti::context;
  using TSti::getOrthogonal;

  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiFPatternStartLoop<SS>>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //TSti::template configure_orthogonal<OrNavigation, CbRotate>(angle);
    TSti::template configure_orthogonal<
      OrNavigation, CbAbsoluteRotate>();  // absolute aligned to the y-axis
    TSti::template configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto & superstate = TSti::template context<SS>();

    auto initialStateAngle = superstate.initialStateAngle;

    float angle = 0;
    double offset = 9;  // for a better behaving

    if (SS::direction() == TDirection::LEFT)
      angle = -offset;
    else
      angle = +offset;

    auto absoluteRotateBehavior =
      TSti::template getOrthogonal<OrNavigation>()->template getClientBehavior<CbAbsoluteRotate>();

    absoluteRotateBehavior->absoluteGoalAngleDegree = initialStateAngle + angle;
    RCLCPP_INFO(
      this->getLogger(), "Fpattern, rotate to: %lf",
      *(absoluteRotateBehavior->absoluteGoalAngleDegree));
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot_strikes_back
