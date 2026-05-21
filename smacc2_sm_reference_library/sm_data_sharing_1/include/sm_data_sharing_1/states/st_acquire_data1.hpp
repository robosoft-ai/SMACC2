// Copyright 2025 Robosoft Inc.
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

namespace sm_data_sharing_1
{
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE: Acquire initial position.
// CbStoreData1 runs on entry and writes initialPosition into CpMissionData.
// The state advances automatically after 5 timer ticks (5s) or on 'N' keypress.
struct StAcquireData1 : smacc2::SmaccState<StAcquireData1, SmDataSharing1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StAcquireData2, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAcquireData2, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrData, cl_data::CbStoreData1>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StAcquireData1] Acquiring initial position (auto-advance in 5s or press N)");
  }
};

}  // namespace sm_data_sharing_1
