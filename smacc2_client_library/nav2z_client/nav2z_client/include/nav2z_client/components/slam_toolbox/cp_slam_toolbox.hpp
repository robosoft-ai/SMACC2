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

#include <rclcpp/rclcpp.hpp>
#include <smacc2/client_bases/smacc_action_client.hpp>
#include <smacc2/component.hpp>
#include <std_msgs/msg/string.hpp>

namespace cl_nav2z
{
// stores the state of the last modification of the slam node
// (blind - open loop solution because the node does not provide that information)
class CpSlamToolbox : public smacc2::ISmaccComponent
{
public:
  CpSlamToolbox();
  virtual ~CpSlamToolbox();

  enum class SlamToolboxState
  {
    Resumed,
    Paused
  };

  inline SlamToolboxState getState() { return state_; }

  void toogleState();

private:
  SlamToolboxState state_;
};
}  // namespace cl_nav2z
