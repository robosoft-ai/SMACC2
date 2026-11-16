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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/client_behaviors/cb_abort_navigation.hpp>
#include <cl_nav2z/common.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

#include <chrono>
#include <thread>

#include <rclcpp/parameter_client.hpp>

namespace cl_nav2z
{
CbAbortNavigation::CbAbortNavigation() {}

void CbAbortNavigation::onEntry()
{
  // connected before the request, so the result of the cancelled goal cannot be missed
  goalFinished_ = false;
  nav2ActionInterface_->onNavigationCancelled(&CbAbortNavigation::onNavigationResult, this);
  nav2ActionInterface_->onNavigationAborted(&CbAbortNavigation::onNavigationResult, this);
  nav2ActionInterface_->onNavigationSucceeded(&CbAbortNavigation::onNavigationResult, this);

  if (!nav2ActionInterface_->cancelNavigation())
  {
    RCLCPP_INFO(getLogger(), "[CbAbortNavigation] no navigation goal in flight");
    this->postSuccessEvent();
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!goalFinished_)
  {
    // the state was left first: no event
    if (this->isShutdownRequested()) return;
    if (std::chrono::steady_clock::now() >= deadline)
    {
      RCLCPP_ERROR(getLogger(), "[CbAbortNavigation] no result for the cancelled goal after 5 s");
      this->postFailureEvent();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(getLogger(), "[CbAbortNavigation] the cancelled goal has finished");
  this->postSuccessEvent();
}

void CbAbortNavigation::onNavigationResult(
  const components::CpNav2ActionInterface::WrappedResult & /*result*/)
{
  goalFinished_ = true;
}

void CbAbortNavigation::onExit() {}

}  // namespace cl_nav2z
