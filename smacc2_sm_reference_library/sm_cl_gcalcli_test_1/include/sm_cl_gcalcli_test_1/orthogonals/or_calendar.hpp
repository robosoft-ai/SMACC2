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

#include <cl_gcalcli/cl_gcalcli.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_cl_gcalcli_test_1
{

class OrCalendar : public smacc2::Orthogonal<OrCalendar>
{
public:
  void onInitialize() override
  {
    // Configure gcalcli with the "Robot" calendar
    cl_gcalcli::GcalcliConfig config;
    config.gcalcli_path = "gcalcli";  // Use system gcalcli from PATH
    config.calendars = {"Robot"};     // Use the Robot calendar
    config.poll_interval = std::chrono::seconds{10};  // Faster polling for testing
    config.heartbeat_interval = std::chrono::seconds{60};
    config.agenda_days = 7;

    this->createClient<cl_gcalcli::ClGcalcli>(config);
  }
};

}  // namespace sm_cl_gcalcli_test_1
