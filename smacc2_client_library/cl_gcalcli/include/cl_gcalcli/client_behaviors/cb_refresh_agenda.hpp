// Copyright 2024 RobosoftAI Inc.
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

#include <smacc2/smacc_client_behavior.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>

namespace cl_gcalcli
{

/**
 * @brief Sync behavior that forces an immediate agenda refresh
 *
 * This behavior triggers an immediate refresh of the calendar agenda
 * without waiting for the next poll interval.
 */
class CbRefreshAgenda : public smacc2::SmaccClientBehavior
{
public:
  CbRefreshAgenda() = default;
  virtual ~CbRefreshAgenda() = default;

  void onEntry() override;

  /**
   * @brief Check if refresh was successful
   */
  bool wasSuccessful() const { return success_; }

private:
  ClGcalcli * client_;
  bool success_;
};

}  // namespace cl_gcalcli
