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

namespace sm_nav2_gazebo_test_2
{
namespace radial_motion_states
{
using namespace cl_keyboard;

// STATE DECLARATION
//
// Retraces the path recorded during StiRadialEndPoint exactly backwards using
// the UndoPathGlobalPlanner + UndoBackwardLocalPlanner pair.
struct StiRadialReturn : smacc2::SmaccState<StiRadialReturn, SS>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS {};
  struct PREVIOUS : ABORT {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StiRadialLoopStart, SUCCESS>,
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StiRadialEndPoint, PREVIOUS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiRadialLoopStart, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // See config/nav2_params.yaml for the plugin instances referenced here
    cl_nav2z::CbUndoPathBackwardsOptions options;
    options.undoControllerName_ = "UndoBackwardLocalPlanner";
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>(options);
  }

  void onExit()
  {
    // Whatever remains of the recorded path is dropped: each iteration starts
    // recording from scratch
    ClNav2Z * navClient;
    this->requiresClient(navClient);

    auto odomTracker = navClient->getComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    odomTracker->clearPath();
  }
};

}  // namespace radial_motion_states
}  // namespace sm_nav2_gazebo_test_2
