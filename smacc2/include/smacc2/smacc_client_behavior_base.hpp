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

#include <string>

#include <smacc2/common.hpp>

namespace smacc2
{
class ISmaccClientBehavior
{
public:
  ISmaccClientBehavior();

  virtual ~ISmaccClientBehavior();

  inline ISmaccStateMachine * getStateMachine();

  std::string getName() const;

  template <typename SmaccClientType>
  void requiresClient(SmaccClientType *& storage);

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage);

protected:
  virtual void runtimeConfigure();

  virtual void onEntry() {}

  virtual void onExit() {}

  template <typename EventType>
  void postEvent(const EventType & ev);

  template <typename EventType>
  void postEvent();

  inline ISmaccState * getCurrentState();

  virtual void dispose();

  virtual rclcpp::Node::SharedPtr getNode();

  virtual rclcpp::Logger getLogger();

private:
  //internal visibility (private + friend)
  virtual void executeOnEntry();

  //internal visibility (private + friend)
  virtual void executeOnExit();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation();

  // a reference to the owner state machine
  ISmaccStateMachine * stateMachine_;

  // a reference to the state where the client behavior is being executed
  ISmaccState * currentState;
  smacc2::ISmaccOrthogonal * currentOrthogonal;

  friend class ISmaccState;
  friend class ISmaccOrthogonal;
  friend class ISmaccAsynchronousClientBehavior;
};
}  // namespace smacc2
