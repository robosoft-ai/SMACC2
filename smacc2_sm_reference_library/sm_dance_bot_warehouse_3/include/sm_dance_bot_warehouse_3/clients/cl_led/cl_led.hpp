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

#include <sm_dance_bot_warehouse_3/action/led_control.hpp>
#include <smacc2/client_bases/smacc_action_client_base.hpp>

namespace sm_dance_bot_warehouse_3
{
namespace cl_led
{
class ClLED
: public smacc2::client_bases::SmaccActionClientBase<sm_dance_bot_warehouse_3::action::LEDControl>
{
public:

  ClLED(std::string actionServerName);
  virtual std::string getName() const override;
  virtual ~ClLED();
};

std::ostream & operator<<(
  std::ostream & out, const sm_dance_bot_warehouse_3::action::LEDControl::Goal & msg);

}  // namespace cl_led
}  // namespace sm_dance_bot_warehouse_3
