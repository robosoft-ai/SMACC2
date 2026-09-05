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

// Continuous behavior: cruise along a fixed heading while the altitude follows a
// sine wave around the entry altitude. Never posts success - the state machine must
// provide the exit event (timer orthogonal, mission event, keyboard). The commanded
// ground track is UNBOUNDED along the heading; pair with an exit event by
// construction and rely on the PX4 geofence as the independent backstop.
// On exit, one final setpoint settles the vehicle at the entry altitude at the last
// commanded ground position. Posts failure only if local position is invalid at entry.
class CbSineAltitudeCruise : public CbPx4ClientBehaviorBase
{
public:
  CbSineAltitudeCruise(
    float groundSpeed = 2.0f, float amplitude = 1.5f, float wavelength = 20.0f,
    float heading = std::numeric_limits<float>::quiet_NaN());

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float groundSpeed_;
  float amplitude_;
  float wavelength_;
  float heading_;

  float startX_ = 0.0f;
  float startY_ = 0.0f;
  float baseZ_ = 0.0f;
  float arcLength_ = 0.0f;
  float lastCmdX_ = 0.0f;
  float lastCmdY_ = 0.0f;
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
