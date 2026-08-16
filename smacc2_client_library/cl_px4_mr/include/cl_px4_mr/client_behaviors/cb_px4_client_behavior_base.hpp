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
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <atomic>
#include <chrono>
#include <optional>

#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_px4_mr
{

// Base for cl_px4_mr client behaviors. Encodes the async-thread discipline
// (see the locking rule in CLAUDE.md): the shared components are resolved and
// completion signals wired in onStateOrthogonalAllocation, on the state
// machine thread - never from the asynchronous onEntry thread, where
// requiresComponent/createSignalConnection contend for the state machine
// mutex and can deadlock against a concurrent transition.
//
// Derived behaviors:
// - override wireCompletionSignals() to connect their completion signal
//   (goal checker reached, vehicle disarmed, ...) - it is invoked on the
//   state machine thread during allocation
// - keep onEntry() as pure command issuance (setpoints, goals, mode switches)
// - finish through postPx4Success()/postPx4Failure(), which latch completion
//   so the timeout watchdog and a late signal can never double-post
//
// Timeout watchdog: setTimeout() arms a deadline checked from update() (the
// SignalDetector thread, ~20 Hz). On expiry the behavior posts failure -
// previously no signal-driven PX4 behavior had ANY failure path, so a stuck
// goal checker or a lost vehicle meant waiting forever. Poll-driven behaviors
// that override update() must chain CbPx4ClientBehaviorBase::update() to keep
// the watchdog.
class CbPx4ClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior,
                                public smacc2::ISmaccUpdatable
{
public:
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // shared component set; SOFT because each behavior uses a subset
    this->requiresComponent(vehicleCommand_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(vehicleStatus_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(trajectorySetpoint_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(goalChecker_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(localPosition_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(offboardKeepAlive_, smacc2::ComponentRequirement::SOFT);

    this->wireCompletionSignals();

    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  virtual ~CbPx4ClientBehaviorBase() {}

  // arm the completion watchdog (checked from update() on the SignalDetector
  // thread); disabled when never called
  void setTimeout(std::chrono::milliseconds timeout) { timeout_ = timeout; }

  void update() override
  {
    if (!timeout_ || completed_)
    {
      return;
    }

    auto now = std::chrono::steady_clock::now();
    if (!watchdogStart_)
    {
      watchdogStart_ = now;
      return;
    }

    if (now - *watchdogStart_ > *timeout_)
    {
      RCLCPP_ERROR(
        getLogger(), "[%s] Timed out after %ld ms without completing - posting failure",
        getName().c_str(), static_cast<long>(timeout_->count()));
      this->postPx4Failure();
    }
  }

protected:
  // connect completion signals here (invoked on the state machine thread
  // during state allocation)
  virtual void wireCompletionSignals() {}

  void postPx4Success()
  {
    if (!completed_.exchange(true))
    {
      this->postSuccessEvent();
    }
  }

  void postPx4Failure()
  {
    if (!completed_.exchange(true))
    {
      this->postFailureEvent();
    }
  }

  CpVehicleCommand * vehicleCommand_ = nullptr;
  CpVehicleStatus * vehicleStatus_ = nullptr;
  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  CpGoalChecker * goalChecker_ = nullptr;
  CpVehicleLocalPosition * localPosition_ = nullptr;
  CpOffboardKeepAlive * offboardKeepAlive_ = nullptr;

private:
  std::atomic<bool> completed_{false};
  std::optional<std::chrono::milliseconds> timeout_;
  std::optional<std::chrono::steady_clock::time_point> watchdogStart_;
};

}  // namespace cl_px4_mr
