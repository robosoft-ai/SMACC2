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

#include <cmath>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpVehicleLocalPosition;

class CbYawRotate : public smacc2::SmaccAsyncClientBehavior, public smacc2::ISmaccUpdatable
{
public:
  CbYawRotate(float targetYawRad, bool relative = false);

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float targetYawRad_;
  bool relative_;
  float absoluteTargetYaw_ = 0.0f;

  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  CpVehicleLocalPosition * localPosition_ = nullptr;
};

}  // namespace cl_px4_mr
