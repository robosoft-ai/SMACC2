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

#pragma once

#include <nav2_msgs/action/assisted_teleop.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{

// Collision-guarded teleoperation via the Nav2 behavior server's
// AssistedTeleop action: incoming teleop velocity commands are projected
// through the local costmap and scaled/zeroed before a collision. A positive
// time allowance bounds the session (the server reports expiry as an abort
// with the TIMEOUT code, which this wrapper maps to EvCbSuccess - the window
// running its course is the normal ending). A zero allowance disables the
// server-side timeout: the session runs until the state exits, which cancels
// the action through the behavior base.
class CbAssistedTeleop : public smacc2::client_behavior_bases::CbActionClientBehaviorBase<
                           nav2_msgs::action::AssistedTeleop>
{
public:
  explicit CbAssistedTeleop(std::chrono::seconds timeAllowance = std::chrono::seconds(30));

  void onEntry() override;

protected:
  // The server reports time-allowance expiry as an abort (error code TIMEOUT)
  // - but for this action the window running its course IS the normal ending,
  // so map it to success. Genuine failures (TF_ERROR, rejection) keep failing.
  void onActionAbort(const WrappedResult & result) override;

private:
  std::chrono::seconds timeAllowance_;
};

}  // namespace cl_nav2z
