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

#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc_state_machine.hpp>

#include <rclcpp_action/client.hpp>

namespace smacc2
{
namespace client_bases
{
// This class interface shows the basic set of methods that
// a SMACC "resource" or "plugin" Action Client has
class ISmaccActionClient : public ISmaccClient
{
public:
  ISmaccActionClient();

  // The destructor. This is called when the object is not
  // referenced anymore by its owner
  virtual ~ISmaccActionClient();

  // Gets the ros path of the action...
  inline std::string getNamespace() const { return name_; }

  virtual void cancelGoal() = 0;

  virtual std::shared_ptr<rclcpp_action::ClientBase> getClientBase() = 0;

protected:
  // The ros path where the action server is located
  std::string name_;
};
}  // namespace client_bases
}  // namespace smacc2
