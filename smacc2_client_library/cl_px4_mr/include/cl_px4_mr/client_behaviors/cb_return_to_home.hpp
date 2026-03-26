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

#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpGoalChecker;

class CbReturnToHome : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbReturnToHome(float homeX, float homeY, float homeZ, float homeYaw);

  void onEntry() override;
  void onExit() override;

private:
  void onGoalReachedCallback();

  float homeX_;
  float homeY_;
  float homeZ_;
  float homeYaw_;

  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  CpGoalChecker * goalChecker_ = nullptr;
};

}  // namespace cl_px4_mr
