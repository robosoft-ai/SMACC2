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

#include <sm_coretest_poll_shutdown_1/clients/cl_slow_update.hpp>
#include <smacc2/smacc.hpp>

namespace sm_coretest_poll_shutdown_1
{
class OrSlow : public smacc2::Orthogonal<OrSlow>
{
public:
  void onInitialize() override { this->createClient<ClSlowUpdate>(); }
};
}  // namespace sm_coretest_poll_shutdown_1
