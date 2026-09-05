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

#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpVehicleLocalPosition;

// Continuous behavior: hold the entry position while the heading sweeps
// sinusoidally around a base heading. Never posts success - the state machine
// must provide the exit event (timer orthogonal, mission event, keyboard).
// On exit, one final setpoint restores the base heading, so the vehicle always
// returns to it regardless of where in the sweep the state change lands.
// Posts failure only if local position is invalid at entry.
class CbYawScan : public CbPx4ClientBehaviorBase
{
public:
  CbYawScan(
    float amplitude = 0.6f, float period = 6.0f,
    float baseHeading = std::numeric_limits<float>::quiet_NaN());

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float amplitude_;
  float period_;
  float baseHeading_;

  float holdX_ = 0.0f;
  float holdY_ = 0.0f;
  float holdZ_ = 0.0f;
  float elapsed_ = 0.0f;
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
