// Copyright 2021 RobosoftAI Inc.
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

#include "ros_timer_client/cl_ros_timer.hpp"
#include "smacc2/smacc.hpp"

namespace $sm_name$
{
using namespace std::chrono_literals;

class OrTimer : public smacc2::Orthogonal<OrTimer>
{
public:
  void onInitialize() override { auto client = this->createClient<cl_ros_timer::ClRosTimer>(1s); }
};
}  // namespace sm_atomic
