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

#include <smacc2/smacc.hpp>

namespace sm_nav2_gazebo_test_2
{
namespace SS1
{
namespace sm_nav2_gazebo_test_2
{
namespace radial_motion_states
{

// FORWARD DECLARATION OF INNER STATES
class StiRadialLoopStart;
class StiRadialRotate;
class StiRadialEndPoint;
class StiRadialReturn;

}  // namespace radial_motion_states
}  // namespace sm_nav2_gazebo_test_2
using namespace sm_nav2_gazebo_test_2::radial_motion_states;
using namespace cl_keyboard;

// STATE DECLARATION
//
// Radial undo pattern: on each iteration the robot rotates to the ray heading,
// navigates forward recording its path with the odom tracker, then retraces the
// exact recorded path backwards with CbUndoPathBackwards.
struct SsRadialPattern1
: smacc2::SmaccState<SsRadialPattern1, SmNav2GazeboTest2, StiRadialLoopStart>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvLoopEnd<StiRadialLoopStart>, StNavigateToFPattern, ENDLOOP>
  > reactions;

  // Pattern geometry: 4 diagonal rays (45, 135, 225, 315 degrees) of 1.5 meters
  // around the pattern center at (0.5, 0.0) - all free space in the open arena
  // world (worlds/open_arena.sdf.xacro)
  static constexpr int total_iterations() { return 4; }
  static constexpr float ray_start_angle_degree() { return 45.0; }
  static constexpr float ray_angle_increment_degree() { return 360.0 / total_iterations(); }
  static constexpr float ray_length_meters() { return 1.5; }

  // Superstate data shared by the inner states across inner transitions
  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Container-level keyboard behavior: one persistent instance serving every
    // inner state (avoids the double-event problem)
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}
};

// ALIAS FOR THE INNER STATES
using SS = SsRadialPattern1;
#include <sm_nav2_gazebo_test_2/states/radial_motion_states/sti_radial_loop_start.hpp>
#include <sm_nav2_gazebo_test_2/states/radial_motion_states/sti_radial_rotate.hpp>
#include <sm_nav2_gazebo_test_2/states/radial_motion_states/sti_radial_end_point.hpp>
#include <sm_nav2_gazebo_test_2/states/radial_motion_states/sti_radial_return.hpp>

}  // namespace SS1
}  // namespace sm_nav2_gazebo_test_2
