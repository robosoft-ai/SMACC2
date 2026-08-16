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

namespace sm_nav2_gazebo_test_3
{
namespace primitive_loop_states
{
using namespace cl_keyboard;

// STATE DECLARATION
//
// Back up home along the leg just driven (odometry-relative, collision-checked)
struct StiBackUp : smacc2::SmaccState<StiBackUp, SS>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbBackUp, OrNavigation>, StiPrimitiveLoopStart, SUCCESS>,
    Transition<EvCbFailure<CbBackUp, OrNavigation>, StiPrimitiveLoopStart, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiPrimitiveLoopStart, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbBackUp>(SS::leg_length_meters(), 0.15f);
  }
};

}  // namespace primitive_loop_states
}  // namespace sm_nav2_gazebo_test_3
