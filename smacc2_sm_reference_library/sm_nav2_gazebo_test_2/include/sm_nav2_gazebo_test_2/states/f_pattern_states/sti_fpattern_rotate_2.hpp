// Copyright 2026 RobosoftAI Inc.
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

namespace sm_nav2_gazebo_test_2
{
namespace f_pattern_states
{

// STATE DECLARATION
//
// Rotate toward the pitch direction (RIGHT pattern: +90 degrees, rows advance
// north)
template <typename SS>
struct StiFPatternRotate2 : smacc2::SmaccState<StiFPatternRotate2<SS>, SS>
{
  typedef smacc2::SmaccState<StiFPatternRotate2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiFPatternForward2<SS>, SUCCESS>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiFPatternRotate2<SS>, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    float angle = (SS::direction() == TDirection::LEFT) ? -90.0 : 90.0;
    TSti::template configure_orthogonal<OrNavigation, CbAbsoluteRotate>(angle);
  }

  void runtimeConfigure()
  {
    auto cbAbsRotate = this->template getClientBehavior<OrNavigation, CbAbsoluteRotate>();
    cbAbsRotate->spinningPlanner = SpinningPlanner::PureSpinning;
  }
};

}  // namespace f_pattern_states
}  // namespace sm_nav2_gazebo_test_2
