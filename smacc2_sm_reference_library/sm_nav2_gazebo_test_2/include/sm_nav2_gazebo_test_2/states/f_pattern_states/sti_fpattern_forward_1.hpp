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
// Navigate the ray forward while the odom tracker records; the recording is
// what StiFPatternReturn1 undoes
template <typename SS>
struct StiFPatternForward1 : public smacc2::SmaccState<StiFPatternForward1<SS>, SS>
{
  typedef smacc2::SmaccState<StiFPatternForward1<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>, SUCCESS>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
  }

  void runtimeConfigure()
  {
    // Force the ray orientation from the previous motion goal so the rows stay
    // parallel even if the robot's heading carries error after the undo return
    // (sm_nav2_test_7 idiom)
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
