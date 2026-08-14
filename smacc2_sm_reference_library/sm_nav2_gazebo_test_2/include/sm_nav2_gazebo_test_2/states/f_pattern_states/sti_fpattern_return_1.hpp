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
// Retrace the ray exactly backwards to the row start
template <typename SS>
struct StiFPatternReturn1 : smacc2::SmaccState<StiFPatternReturn1<SS>, SS>
{
  typedef smacc2::SmaccState<StiFPatternReturn1<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StiFPatternRotate2<SS>, SUCCESS>,
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StiFPatternRotate2<SS>, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    cl_nav2z::CbUndoPathBackwardsOptions options;
    options.undoControllerName_ = "UndoBackwardLocalPlanner";
    TSti::template configure_orthogonal<OrNavigation, CbUndoPathBackwards>(options);
  }

  void onExit()
  {
    // Each row starts recording from scratch
    ClNav2Z * navClient;
    this->requiresClient(navClient);

    auto odomTracker = navClient->template getComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    odomTracker->clearPath();
  }
};

}  // namespace f_pattern_states
}  // namespace sm_nav2_gazebo_test_2
