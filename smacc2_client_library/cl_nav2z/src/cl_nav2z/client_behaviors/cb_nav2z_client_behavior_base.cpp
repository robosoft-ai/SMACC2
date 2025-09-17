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
#include <cl_nav2z/client_behaviors/cb_nav2z_client_behavior_base.hpp>
#include <cl_nav2z/common.hpp>

namespace cl_nav2z
{
CbNav2ZClientBehaviorBase::~CbNav2ZClientBehaviorBase() {}

// REMOVED: All legacy implementation - behaviors now use component APIs directly
// The base class provides component access but no complex logic
// Individual behaviors should use:
//   nav2ActionInterface_->sendGoal(goal)
//   nav2ActionInterface_->onNavigationSucceeded(&Behavior::onSuccess, this)
//   etc.

}  // namespace cl_nav2z
