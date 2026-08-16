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

class CbSpiralPattern : public CbPx4ClientBehaviorBase
{
public:
  CbSpiralPattern(
    float centerX, float centerY, float altitude, float maxRadius = 20.0f, float spacing = 3.0f,
    float speed = 2.0f);

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float centerX_;
  float centerY_;
  float altitude_;
  float maxRadius_;
  float spacing_;
  float speed_;

  float theta_ = 0.0f;
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
