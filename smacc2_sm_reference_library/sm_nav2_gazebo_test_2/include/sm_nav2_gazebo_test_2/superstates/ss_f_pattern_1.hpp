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
namespace f_pattern_states
{
enum class TDirection
{
  LEFT,
  RIGHT
};

// FORWARD DECLARATIONS OF INNER STATES (templated on the superstate, so the
// same pattern states can serve multiple F pattern superstates - the
// sm_nav2_test_7 idiom)
template <typename SS>
class StiFPatternStartLoop;
template <typename SS>
class StiFPatternRotate1;
template <typename SS>
class StiFPatternForward1;
template <typename SS>
class StiFPatternReturn1;
template <typename SS>
class StiFPatternRotate2;
template <typename SS>
class StiFPatternForward2;

}  // namespace f_pattern_states

namespace SS2
{
using namespace f_pattern_states;

// STATE DECLARATION
//
// Boustrophedon "F" pattern with undo returns (ported from nova_carter
// sm_nav2_test_7 SsFPattern1): each iteration rotates to the ray heading,
// navigates the ray forward recording the path, retraces it exactly backwards
// with CbUndoPathBackwards, then rotates and advances one pitch to the next row.
struct SsFPattern1 : smacc2::SmaccState<SsFPattern1, SmNav2GazeboTest2, StiFPatternStartLoop<SsFPattern1>>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvLoopEnd<StiFPatternStartLoop<SsFPattern1>>, StFinalSpin, ENDLOOP>
  > reactions;

  // Pattern geometry: 3 east-pointing rays of 1.2 m, rows pitched 0.4 m north,
  // starting from (0.0, -1.5) - free space verified against walls and pillars
  static constexpr float ray_length_meters() { return 1.2; }
  static constexpr float pitch_length_meters() { return 0.4; }
  static constexpr int total_iterations() { return 3; }
  static constexpr TDirection direction() { return TDirection::RIGHT; }

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

}  // namespace SS2
}  // namespace sm_nav2_gazebo_test_2

#include <sm_nav2_gazebo_test_2/states/f_pattern_states/sti_fpattern_loop_start.hpp>
#include <sm_nav2_gazebo_test_2/states/f_pattern_states/sti_fpattern_rotate_1.hpp>
#include <sm_nav2_gazebo_test_2/states/f_pattern_states/sti_fpattern_forward_1.hpp>
#include <sm_nav2_gazebo_test_2/states/f_pattern_states/sti_fpattern_return_1.hpp>
#include <sm_nav2_gazebo_test_2/states/f_pattern_states/sti_fpattern_rotate_2.hpp>
#include <sm_nav2_gazebo_test_2/states/f_pattern_states/sti_fpattern_forward_2.hpp>
