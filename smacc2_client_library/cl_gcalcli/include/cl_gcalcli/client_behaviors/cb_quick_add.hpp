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

#include <string>

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>

namespace cl_gcalcli
{

/**
 * @brief Async behavior that adds an event using gcalcli quick add
 *
 * This behavior uses gcalcli's "quick" command to add events using
 * natural language. Posts success on completion or failure on error.
 *
 * Example:
 * @code
 * configure<OrCalendar, CbQuickAdd>("Team meeting tomorrow at 3pm for 1 hour");
 * @endcode
 */
class CbQuickAdd : public smacc2::SmaccAsyncClientBehavior
{
public:
  /**
   * @brief Construct with event description
   * @param text Natural language event description
   * @param calendar Optional specific calendar name
   */
  explicit CbQuickAdd(const std::string & text, const std::string & calendar = "");

  virtual ~CbQuickAdd() = default;

  void onEntry() override;

private:
  std::string text_;
  std::string calendar_;
  ClGcalcli * client_;
};

}  // namespace cl_gcalcli
