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

#include <nitrosz_client/client_behaviors/cb_abort_navigation.hpp>
#include <nitrosz_client/common.hpp>
#include <nitrosz_client/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <nitrosz_client/components/odom_tracker/cp_odom_tracker.hpp>
#include <nitrosz_client/components/pose/cp_pose.hpp>
#include <nitrosz_client/nitrosz_client.hpp>

#include <rclcpp/parameter_client.hpp>

namespace cl_nitrosz
{
CbAbortNavigation::CbAbortNavigation() {}

void CbAbortNavigation::onEntry()
{
  // this->sendGoal(goal);

  this->nitroszClient_->cancelGoal();
  this->postSuccessEvent();
}

void CbAbortNavigation::onExit() {}

}  // namespace cl_nitrosz
