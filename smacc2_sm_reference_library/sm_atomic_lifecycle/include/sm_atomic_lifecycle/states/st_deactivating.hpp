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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc2/smacc.hpp>

namespace sm_atomic_lifecycle
{
using sm_atomic_lifecycle::OrLifecycleNode;
using namespace cl_lifecyclenode;

// STATE DECLARATION
struct StDeactivating : smacc2::SmaccState<StDeactivating, SmAtomicLifecycle>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
          Transition<EvTransitionOnDeactivateSuccess<ClLifecycleNode, OrLifecycleNode>, StInactive, SUCCESS>,
          Transition<EvTransitionOnDeactivateError<ClLifecycleNode, OrLifecycleNode>, StErrorProcessing, ABORT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "On Entry!");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "On Exit!");
  }
};
}  // namespace sm_atomic_lifecycle
