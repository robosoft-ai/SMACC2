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

#include <sm_coretest_post_during_exit_1/clients/cl_pulse.hpp>
#include <smacc2/smacc.hpp>

namespace sm_coretest_post_during_exit_1
{
class OrPulse : public smacc2::Orthogonal<OrPulse>
{
public:
  void onInitialize() override { this->createClient<ClPulse>(); }
};

// Hosts CbSleepFor, which needs no client.
class OrSleep : public smacc2::Orthogonal<OrSleep>
{
public:
  void onInitialize() override {}
};
}  // namespace sm_coretest_post_during_exit_1
