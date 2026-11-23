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

#include <chrono>
#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>
#include <cmath>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;

struct FigureEightParams
{
  float centerX = 0.0f;   // NED north, m
  float centerY = 0.0f;   // NED east, m
  float altitude = 5.0f;  // m above the NED origin
  float size = 5.0f;      // m from the centre to a lobe tip
  float speed = 0.5f;     // rate of the curve parameter, rad/s
  int numLoops = 1;
};

class CbFigureEight : public CbPx4ClientBehaviorBase
{
public:
  explicit CbFigureEight(const FigureEightParams & params);

  // Override the centre before onEntry (e.g. from the owning state's
  // runtimeConfigure when the centre is only known at runtime).
  void setCenter(float centerX, float centerY)
  {
    centerX_ = centerX;
    centerY_ = centerY;
  }
  // Rotate the lobe axis (NED yaw, radians; default 0 = lobes north-south).
  void setHeading(float headingRad) { heading_ = headingRad; }

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float centerX_;
  float centerY_;
  float altitude_;
  float size_;
  float speed_;
  int numLoops_;
  float heading_ = 0.0f;

  float t_ = 0.0f;
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
