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

#pragma once

#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>

#include <chrono>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;

class CbHoldPosition : public CbPx4ClientBehaviorBase
{
public:
  explicit CbHoldPosition(float durationSeconds = 5.0f);

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float durationSeconds_;
  std::chrono::steady_clock::time_point startTime_;
};

}  // namespace cl_px4_mr
