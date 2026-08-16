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
// Full in-place rotation, collision-checked by the behavior server
struct StiSpin : smacc2::SmaccState<StiSpin, SS>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSpin, OrNavigation>, StiBackUp, SUCCESS>,
    Transition<EvCbFailure<CbSpin, OrNavigation>, StiBackUp, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiBackUp, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbSpin>(2 * M_PI);
  }

  void runtimeConfigure()
  {
    // alternate the spin direction each lap: odd iterations counter-clockwise,
    // even iterations clockwise (exercises negative target yaw + runtime goal
    // configuration between staticConfigure and onEntry)
    auto & superstate = this->context<SS>();
    auto cbSpin = this->getClientBehavior<OrNavigation, CbSpin>();
    float direction = (superstate.iteration_count % 2 == 1) ? 1.0f : -1.0f;
    cbSpin->setTargetYaw(direction * 2 * M_PI);
  }
};

}  // namespace primitive_loop_states
}  // namespace sm_nav2_gazebo_test_3
