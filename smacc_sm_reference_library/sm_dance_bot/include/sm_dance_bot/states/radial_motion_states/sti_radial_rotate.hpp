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
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialRotate : smacc2::SmaccState<StiRadialRotate, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiRadialEndPoint, SUCCESS>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiRadialLoopStart, ABORT>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto cbAbsRotate = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbAbsoluteRotate>();

    cbAbsRotate->spinningPlanner = SpiningPlanner::PureSpinning;

    auto & superstate = this->context<SS>();
    cbAbsRotate->absoluteGoalAngleDegree =
      superstate.iteration_count * SS::ray_angle_increment_degree();
  }
};
}  // namespace radial_motion_states
}  // namespace sm_dance_bot
