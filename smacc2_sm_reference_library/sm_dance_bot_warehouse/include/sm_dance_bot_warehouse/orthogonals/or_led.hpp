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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <sm_dance_bot_warehouse/clients/cl_led/cl_led.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_dance_bot_warehouse
{
class OrLED : public smacc2::Orthogonal<OrLED>
{
public:
  void onInitialize() override
  {
    auto actionclient = this->createClient<cl_led::ClLED>("led_action_server");
  }
};
}  // namespace sm_dance_bot_warehouse
