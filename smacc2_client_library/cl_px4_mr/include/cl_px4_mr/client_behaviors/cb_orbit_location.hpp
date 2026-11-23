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
#include <cmath>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpVehicleLocalPosition;

struct OrbitLocationParams
{
  float centerX = 0.0f;          // NED north, m
  float centerY = 0.0f;          // NED east, m
  float altitude = 5.0f;         // m above the NED origin
  float radius = 5.0f;           // m
  float angularVelocity = 0.5f;  // rad/s
  int numOrbits = 3;
};

class CbOrbitLocation : public CbPx4ClientBehaviorBase
{
public:
  explicit CbOrbitLocation(const OrbitLocationParams & params);

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float centerX_;
  float centerY_;
  float altitude_;
  float radius_;
  float angularVelocity_;
  int numOrbits_;

  float currentAngle_ = 0.0f;
  float startAngle_ = 0.0f;
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
