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

#include <smacc2/common.hpp>
#include <smacc2/component.hpp>
#include <typeinfo>

namespace smacc2
{
struct ComponentKey
{
  ComponentKey(const std::type_info * typeinfo, std::string name)
  {
    this->name = name;
    this->typeinfo = typeinfo;
    encodedKey = std::to_string((long)(void *)typeinfo) + "_" + name;
  }
  std::string encodedKey;
  const std::type_info * typeinfo;
  std::string name;

  bool operator<(const ComponentKey & other) const { return this->encodedKey < other.encodedKey; }
  bool operator==(const ComponentKey & other) const { return this->encodedKey == other.encodedKey; }
};

class ISmaccClient
{
public:
  ISmaccClient();
  virtual ~ISmaccClient();

  virtual void onInitialize();

  // Returns a custom identifier defined by the specific plugin implementation
  virtual std::string getName() const;

  template <typename TComponent>
  TComponent * getComponent();

  template <typename TComponent>
  TComponent * getComponent(std::string name);

  // Gets the i-th component of type TComponent
  template <typename TComponent>
  TComponent * getComponent(int index);

  virtual smacc2::introspection::TypeInfo::Ptr getType();

  inline ISmaccStateMachine * getStateMachine();

  template <typename TSmaccSignal, typename T>
  void connectSignal(TSmaccSignal & signal, void (T::*callback)(), T * object);

  template <typename SmaccClientType>
  void requiresClient(SmaccClientType *& storage);

  void getComponents(std::vector<std::shared_ptr<ISmaccComponent>> & components);

  // now this needs to be public because sub-components needs to use. This is something to improve.
  template <typename EventType>
  void postEvent(const EventType & ev);

  // now this needs to be public because sub-components needs to use. This is something to improve.
  template <typename EventType>
  void postEvent();

protected:
  // it is called after the client initialization, provides information about the orthogonal it is located in
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
  }

  // components
  std::map<ComponentKey, std::shared_ptr<smacc2::ISmaccComponent>> components_;

  template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
  SmaccComponentType * createComponent(TArgs... targs);

  template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
  SmaccComponentType * createNamedComponent(std::string name, TArgs... targs);

  rclcpp::Node::SharedPtr getNode();

  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }

private:
  // A reference to the state machine object that owns this resource
  ISmaccStateMachine * stateMachine_;
  ISmaccOrthogonal * orthogonal_;

  // friend method called by orthogonal
  void initialize();

  // friend method called by orthogonal
  // Assigns the owner of this resource to the given state machine parameter object
  void setStateMachine(ISmaccStateMachine * stateMachine);

  // friend method called by orthogonal
  void setOrthogonal(ISmaccOrthogonal * orthogonal);

  friend class ISmaccOrthogonal;
  friend class ISmaccComponent;
};
}  // namespace smacc2
