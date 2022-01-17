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

#include <boost/optional.hpp>
#include <smacc2/common.hpp>

namespace smacc2
{
class ISmaccComponent
{
public:
  ISmaccComponent();

  virtual ~ISmaccComponent();

  // Returns a custom identifier defined by the specific plugin implementation
  virtual std::string getName() const;

protected:
  // this is the basic initialization method that each specific component should
  // implement. The owner and the node are already available when it is invoked by the client.
  virtual void onInitialize();

  template <typename EventType>
  void postEvent(const EventType & ev);

  template <typename EventType>
  void postEvent();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
  }

  template <typename TComponent>
  void requiresComponent(TComponent *& requiredComponentStorage);

  template <typename TClient>
  void requiresClient(TClient *& requiredClientStorage);

  template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
  SmaccComponentType * createSiblingComponent(TArgs... targs);

  template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
  SmaccComponentType * createSiblingNamedComponent(std::string name, TArgs... targs);

  rclcpp::Node::SharedPtr getNode();

  rclcpp::Logger getLogger();

  // A reference to the state machine object that owns this resource
  ISmaccStateMachine * stateMachine_;

  ISmaccClient * owner_;

private:
  // friend method invoked by the client.
  void initialize(ISmaccClient * owner);

  // friend method invoked by the client.
  // Assigns the owner of this resource to the given state machine parameter object
  void setStateMachine(ISmaccStateMachine * stateMachine);

  friend class ISmaccOrthogonal;
  friend class ISmaccClient;
};
}  // namespace smacc2
