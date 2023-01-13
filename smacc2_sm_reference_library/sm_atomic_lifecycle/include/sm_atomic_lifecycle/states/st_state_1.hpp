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

namespace sm_atomic_lifecycle
{
using namespace cl_ros_timer;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
using cl_lifecyclenode::CbConfigure;
using cl_lifecyclenode::EvTransitionConfigure;

struct State1 : smacc2::SmaccState<State1, SmAtomicLifecycle>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    // Transition<EvTransitionConfigure<CbConfigure, OrLifecycleNode>, State2, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrLifecycleNode, CbConfigure>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_atomic_lifecycle
