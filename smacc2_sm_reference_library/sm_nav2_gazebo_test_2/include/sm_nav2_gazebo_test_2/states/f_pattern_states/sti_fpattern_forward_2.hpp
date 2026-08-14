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
// Advance one pitch to the next row, then close the loop
template <typename SS>
struct StiFPatternForward2 : smacc2::SmaccState<StiFPatternForward2<SS>, SS>
{
  typedef smacc2::SmaccState<StiFPatternForward2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternStartLoop<SS>, SUCCESS>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiFPatternStartLoop<SS>, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(
      SS::pitch_length_meters());
  }

  void runtimeConfigure()
  {
    // Keep the pitch direction consistent with the just-completed rotation goal
    ClNav2Z * navClient;
    this->requiresClient(navClient);
    auto odomTracker = navClient->template getComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    auto * cbForwardMotion = this->template getOrthogonal<OrNavigation>()
                               ->template getClientBehavior<CbNavigateForward>();
    auto previousGoal = odomTracker->getCurrentMotionGoal();

    if (previousGoal)
    {
      cbForwardMotion->options.forceInitialOrientation = previousGoal->pose.orientation;
    }
  }
};

}  // namespace f_pattern_states
}  // namespace sm_nav2_gazebo_test_2
