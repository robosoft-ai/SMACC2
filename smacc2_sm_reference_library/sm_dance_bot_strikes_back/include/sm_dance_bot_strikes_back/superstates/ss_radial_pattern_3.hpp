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

namespace sm_dance_bot_strikes_back
{
namespace SS3
{
namespace sm_dance_bot_strikes_back
{
namespace radial_motion_states
{
// FORWARD DECLARATION OF INNER STATES
class StiRadialRotate;
class StiRadialReturn;
class StiRadialEndPoint;
class StiRadialLoopStart;
}  // namespace radial_motion_states
}  // namespace sm_dance_bot_strikes_back
using namespace sm_dance_bot_strikes_back::radial_motion_states;

// STATE DECLARATION
struct SsRadialPattern3
: smacc2::SmaccState<SsRadialPattern3, MsDanceBotRunMode, StiRadialLoopStart>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopEnd<StiRadialLoopStart>, StNavigateToWaypointsX, ENDLOOP>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  int iteration_count;

  static constexpr int total_iterations() { return 32; }
  static constexpr float ray_angle_increment_degree() { return 360.0 / total_iterations(); }
  static constexpr float ray_length_meters() { return 4; }

  void runtimeConfigure() { iteration_count = 0; }
};

// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsRadialPattern3;
#include <sm_dance_bot_strikes_back/states/radial_motion_states/sti_radial_end_point.hpp>
#include <sm_dance_bot_strikes_back/states/radial_motion_states/sti_radial_loop_start.hpp>
#include <sm_dance_bot_strikes_back/states/radial_motion_states/sti_radial_return.hpp>
#include <sm_dance_bot_strikes_back/states/radial_motion_states/sti_radial_rotate.hpp>
}  // namespace SS3
}  // namespace sm_dance_bot_strikes_back
