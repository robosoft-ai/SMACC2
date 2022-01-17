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
#include <utility>

namespace smacc2
{
class ISmaccOrthogonal
{
public:
  void setStateMachine(ISmaccStateMachine * value);

  inline ISmaccStateMachine * getStateMachine();

  void addClientBehavior(std::shared_ptr<smacc2::ISmaccClientBehavior> clientBehavior);

  void runtimeConfigure();

  void onEntry();

  void onExit();

  virtual std::string getName() const;

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage);

  template <typename SmaccClientType>
  bool requiresClient(SmaccClientType *& storage);

  inline const std::vector<std::shared_ptr<smacc2::ISmaccClient>> & getClients();

  inline const std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>> & getClientBehaviors()
    const;

  template <typename T>
  void setGlobalSMData(std::string name, T value);

  template <typename T>
  bool getGlobalSMData(std::string name, T & ret);

  template <typename TClientBehavior>
  TClientBehavior * getClientBehavior();

  rclcpp::Node::SharedPtr getNode();
  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }

protected:
  virtual void onInitialize();

  void initializeClients();

  template <typename TOrthogonal, typename TClient>
  void assignClientToOrthogonal(TClient * client);

  std::vector<std::shared_ptr<smacc2::ISmaccClient>> clients_;

private:
  ISmaccStateMachine * stateMachine_;

  std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>> clientBehaviors_;
  friend class ISmaccStateMachine;
};

}  // namespace smacc2
