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

#include <chrono>
#include <cmath>
#include <limits>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpVehicleLocalPosition;

// Helix about a vertical axis: a constant-radius orbit that climbs
// climbPerOrbitM every revolution until climbTotalM has been gained, nose
// toward the axis. The orbit starts at the vehicle's current bearing from the
// centre (fly to a point on the circle first, e.g. with CbGoToLocation) and at
// startAltitudeM (NaN = the current altitude). Posts success at the top, where
// the vehicle is on the circle at startAltitude + climbTotal - hand over to
// CbOrbitLocation at that altitude for level laps.
struct SpiralUpParams
{
  float centerX = 0.0f;  // NED north
  float centerY = 0.0f;  // NED east
  float radiusM = 6.0f;
  float startAltitudeM = std::numeric_limits<float>::quiet_NaN();  // above the NED origin
  float climbTotalM = 24.0f;
  float climbPerOrbitM = 3.0f;
  float angularVelocityRadS = 0.3f;  // +ve = clockwise seen from above (NED angle north -> east)
};

class CbSpiralUp : public CbPx4ClientBehaviorBase
{
public:
  explicit CbSpiralUp(SpiralUpParams params = {});

  void onEntry() override;
  void onExit() override {}
  void update() override;

  const SpiralUpParams & params() const { return params_; }

private:
  void command();

  SpiralUpParams params_;
  float startAltitude_ = 0.0f;
  float startAngle_ = 0.0f;
  float currentAngle_ = 0.0f;
  int lastReportedOrbit_ = -1;
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
