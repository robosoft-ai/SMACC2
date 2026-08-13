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
// Navigates forward along the current ray while CbNavigateForward puts the odom
// tracker in RECORD_PATH mode: this recorded path is what StiRadialReturn undoes.
struct StiRadialEndPoint : smacc2::SmaccState<StiRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiRadialReturn, SUCCESS>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiRadialReturn, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiRadialReturn, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(
      getLogger(), "[StiRadialEndPoint] navigating forward %lf meters (recording path)",
      static_cast<double>(SS::ray_length_meters()));
  }
};

}  // namespace radial_motion_states
}  // namespace sm_nav2_gazebo_test_2
