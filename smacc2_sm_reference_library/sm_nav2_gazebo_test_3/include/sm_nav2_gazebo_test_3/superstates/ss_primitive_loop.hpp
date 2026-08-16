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

namespace sm_nav2_gazebo_test_3
{
namespace SS1
{
namespace sm_nav2_gazebo_test_3
{
namespace primitive_loop_states
{

// FORWARD DECLARATION OF INNER STATES
class StiPrimitiveLoopStart;
class StiDriveOnHeading;
class StiSpin;
class StiBackUp;

}  // namespace primitive_loop_states
}  // namespace sm_nav2_gazebo_test_3
using namespace sm_nav2_gazebo_test_3::primitive_loop_states;
using namespace cl_keyboard;

// STATE DECLARATION
//
// Loops the three collision-checked behavior server primitives: drive out on
// the current heading, spin a full turn in place, back up home. Repeated goal
// cycles per CpActionClient component across state re-entries exercise the
// CbActionClientBehaviorBase lifecycle (one signal wiring per behavior
// instance, fresh instances every iteration).
struct SsPrimitiveLoop : smacc2::SmaccState<SsPrimitiveLoop, SmNav2GazeboTest3, StiPrimitiveLoopStart>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvLoopEnd<StiPrimitiveLoopStart>, StSpinToWall, ENDLOOP>
  > reactions;

  static constexpr int total_iterations() { return 3; }
  static constexpr float leg_length_meters() { return 1.2; }

  // superstate data shared by the inner states
  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Container-level keyboard behavior (double-event problem avoidance)
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() { iteration_count = 0; }
};

// ALIAS FOR THE INNER STATES
using SS = SsPrimitiveLoop;
#include <sm_nav2_gazebo_test_3/states/primitive_loop_states/sti_primitive_loop_start.hpp>
#include <sm_nav2_gazebo_test_3/states/primitive_loop_states/sti_drive_on_heading.hpp>
#include <sm_nav2_gazebo_test_3/states/primitive_loop_states/sti_spin.hpp>
#include <sm_nav2_gazebo_test_3/states/primitive_loop_states/sti_back_up.hpp>

}  // namespace SS1
}  // namespace sm_nav2_gazebo_test_3
