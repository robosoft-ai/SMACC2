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

#include <functional>

#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace cl_px4_mr
{

// Turns CpForwardObstacleGuard's signals into state machine events
// EvObstacleTooClose / EvObstacleCleared<CbObstacleGuard, TOrthogonal>.
// Synchronous behavior: onEntry runs on the state machine thread, so it may
// resolve the component and create signal connections there. Configure it
// once on a container state (e.g. the in-flight mode state) so a single
// instance covers every inner flight state.
class CbObstacleGuard : public smacc2::SmaccClientBehavior
{
public:
  CbObstacleGuard();
  virtual ~CbObstacleGuard();

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    postTooClose_ = [this]() { this->postEvent<EvObstacleTooClose<TSourceObject, TOrthogonal>>(); };
    postCleared_ = [this]() { this->postEvent<EvObstacleCleared<TSourceObject, TOrthogonal>>(); };
  }

  void onEntry() override;
  void onExit() override;

private:
  void onTooClose();
  void onCleared();

  CpForwardObstacleGuard * guard_ = nullptr;
  std::function<void()> postTooClose_;
  std::function<void()> postCleared_;
};

}  // namespace cl_px4_mr
