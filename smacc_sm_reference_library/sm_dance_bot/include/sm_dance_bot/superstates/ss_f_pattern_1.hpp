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

#include <smacc2/smacc.hpp>

namespace sm_dance_bot
{
namespace f_pattern_states
{
enum class TDirection
{
  LEFT,
  RIGHT
};

// FORWARD DECLARATIONS OF INNER STATES
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
template <typename SS>
class StiFPatternStartLoop;
}  // namespace f_pattern_states
}  // namespace sm_dance_bot
namespace sm_dance_bot
{
namespace SS4
{
using namespace f_pattern_states;

// STATE DECLARATION
struct SsFPattern1
: smacc2::SmaccState<SsFPattern1, MsDanceBotRunMode, StiFPatternStartLoop<SsFPattern1>>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopEnd<StiFPatternStartLoop<SsFPattern1>>, StNavigateForward2, ENDLOOP>  //,

    >
    reactions;

  // STATE VARIABLES
  // superstate parameters
  static constexpr float ray_lenght_meters() { return 3.25; }
  static constexpr float pitch_lenght_meters() { return 0.75; }
  static constexpr int total_iterations() { return 10; }
  static constexpr TDirection direction() { return TDirection::RIGHT; }

  // superstate state variables
  int iteration_count;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure() { iteration_count = 0; }
};  // namespace SS4

// FORWARD DECLARATION FOR THE SUPERSTATE

}  // namespace SS4
}  // namespace sm_dance_bot

#include <sm_dance_bot/states/f_pattern_states/sti_fpattern_forward_1.hpp>
#include <sm_dance_bot/states/f_pattern_states/sti_fpattern_forward_2.hpp>
#include <sm_dance_bot/states/f_pattern_states/sti_fpattern_loop_start.hpp>
#include <sm_dance_bot/states/f_pattern_states/sti_fpattern_return_1.hpp>
#include <sm_dance_bot/states/f_pattern_states/sti_fpattern_rotate_1.hpp>
#include <sm_dance_bot/states/f_pattern_states/sti_fpattern_rotate_2.hpp>
