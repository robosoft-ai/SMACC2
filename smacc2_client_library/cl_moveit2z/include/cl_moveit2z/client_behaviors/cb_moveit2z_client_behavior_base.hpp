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

/*****************************************************************************************************************
 *
 * 	 Authors: Brett Aldrich, Pablo Inigo Blasco
 *
 ******************************************************************************************************************/

#pragma once

#include <cl_moveit2z/components/cp_motion_planner.hpp>
#include <cl_moveit2z/components/cp_move_group_interface.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <cl_moveit2z/components/cp_trajectory_history.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_moveit2z
{

// Base for cl_moveit2z motion behaviors. Encodes the async-thread discipline
// (see the locking rule in CLAUDE.md): all components are resolved here, in
// onStateOrthogonalAllocation on the state machine thread - never from the
// asynchronous onEntry/onExit threads, where requiresComponent contends for
// the state machine mutex and can deadlock against a concurrent transition.
// Derived behaviors use the protected component members directly and finish
// through postMotionSuccess()/postMotionFailure(), which centralize the
// client-scoped motion event + behavior-scoped EvCbSuccess/EvCbFailure pair.
//
// Execution model is unchanged: blocking MoveGroupInterface calls on the
// asynchronous behavior thread (check isShutdownRequested() in long loops).
class CbMoveit2zClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior
{
public:
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // CpMoveGroupInterface is always created by ClMoveit2z; the rest are
    // optional per-SM opt-ins (preferred path when present, legacy fallback
    // otherwise - same semantics the behaviors implemented individually)
    this->requiresComponent(cpMoveGroup_, smacc2::ComponentRequirement::HARD);
    this->requiresComponent(cpMotionPlanner_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(cpTrajectoryExecutor_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(cpTrajectoryHistory_, smacc2::ComponentRequirement::SOFT);

    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  virtual ~CbMoveit2zClientBehaviorBase() {}

protected:
  void postMotionSuccess()
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] motion execution succeeded. Throwing success event.");
    if (cpMoveGroup_ != nullptr)
    {
      cpMoveGroup_->postEventMotionExecutionSucceeded();
    }
    this->postSuccessEvent();
  }

  void postMotionFailure()
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << getName() << "] motion execution failed. Throwing fail event.");
    if (cpMoveGroup_ != nullptr)
    {
      cpMoveGroup_->postEventMotionExecutionFailed();
    }
    this->postFailureEvent();
  }

  CpMoveGroupInterface * cpMoveGroup_ = nullptr;
  CpMotionPlanner * cpMotionPlanner_ = nullptr;
  CpTrajectoryExecutor * cpTrajectoryExecutor_ = nullptr;
  CpTrajectoryHistory * cpTrajectoryHistory_ = nullptr;
};

}  // namespace cl_moveit2z
