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
#include "sm_autoware_avp/clients/autoware_client/cl_autoware.hpp"

namespace sm_autoware_avp
{
using namespace std::chrono_literals;

class OrAutowareAuto : public smacc2::Orthogonal<OrAutowareAuto>
{
public:
  void onInitialize() override 
  { 
      auto client = this->createClient<sm_autoware_avp::clients::ClAutoware>(); 
      auto cppub = client->createComponent<smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped>>();
      auto cppsub = client->createComponent<smacc2::components::CpTopicSubscriber<geometry_msgs::msg::PoseStamped>>();
  }
};
}  // namespace sm_atomic
