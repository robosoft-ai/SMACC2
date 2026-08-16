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

#include <atomic>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommand;
class CpVehicleStatus;
class CpOffboardKeepAlive;

class CbArmPX4 : public CbPx4ClientBehaviorBase
{
public:
  CbArmPX4();

  void onEntry() override;

  void wireCompletionSignals() override;
  void onExit() override;

private:
  void onArmedCallback();
  std::atomic<bool> armed_{false};
  static constexpr int MAX_RETRIES = 5;
  static constexpr int RETRY_INTERVAL_SEC = 5;
};

}  // namespace cl_px4_mr
