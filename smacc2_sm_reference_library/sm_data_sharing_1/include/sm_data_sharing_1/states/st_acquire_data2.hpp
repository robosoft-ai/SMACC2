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

// STATE: Acquire target position.
// CbStoreData2 runs on entry and writes targetPosition into CpMissionData.
// initialPosition set in StAcquireData1 is still in the component — it persists
// because CpMissionData is state-machine-scoped (lives on ClData, not on this state).
struct StAcquireData2 : smacc2::SmaccState<StAcquireData2, SmDataSharing1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StProcessData, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StProcessData, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrData, cl_data::CbStoreData2>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StAcquireData2] Acquiring target position (auto-advance in 5s or press N)");
  }
};

}  // namespace sm_data_sharing_1
