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
#include <sm_dance_bot_warehouse_3/clients/cl_led/cl_led.hpp>
//#include <pluginlib/class_list_macros.h>

namespace sm_dance_bot_warehouse_3
{
namespace cl_led
{
ClLED::ClLED(std::string actionServerName)
: SmaccActionClientBase<sm_dance_bot_warehouse_3::action::LEDControl>(actionServerName)
{
}

std::string ClLED::getName() const { return "TOOL ACTION CLIENT"; }

ClLED::~ClLED() {}

std::ostream & operator<<(
  std::ostream & out, const sm_dance_bot_warehouse_3::action::LEDControl::Goal & msg)
{
  out << "LED CONTROL: " << msg.command;
  return out;
}

}  // namespace cl_led

//PLUGINLIB_EXPORT_CLASS(cl_led::ClLED, smacc2::ISmaccComponent)
}  // namespace sm_dance_bot_warehouse_3
