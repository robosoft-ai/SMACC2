// Copyright 2025 Robosoft Inc.
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
  virtual ~ISmaccOrthogonal() = default;

  void setStateMachine(ISmaccStateMachine * value);

  inline ISmaccStateMachine * getStateMachine();

  void initState(ISmaccState * state);

  void addClientBehavior(std::shared_ptr<smacc2::ISmaccClientBehavior> clBehavior);

  void runtimeConfigure();

  void onEntry();

  void onExit();

  void onDispose();

  virtual std::string getName() const;

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage);

  template <typename SmaccClientType>
  bool requiresClient(SmaccClientType *& storage);

  inline const std::vector<std::shared_ptr<smacc2::ISmaccClient>> & getClients();

  inline const std::vector<std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>>> &
  getClientBehaviors() const;

  template <typename T>
  void setGlobalSMData(std::string name, T value);

  template <typename T>
  bool getGlobalSMData(std::string name, T & ret);

  // returns a client behavior of a given type. If the client behavior is not found, it returns nullptr
  // the index parameter is used to specify the client behavior in case there are more than one
  template <typename TClientBehavior>
  TClientBehavior * getClientBehavior(int index = 0);

  rclcpp::Node::SharedPtr getNode();
  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }

protected:
  virtual void onInitialize();

  void initializeClients();

  template <typename TOrthogonal, typename TClient>
  void assignClientToOrthogonal(TClient * client);

  std::vector<std::shared_ptr<smacc2::ISmaccClient>> clients_;

private:
  // Null until setStateMachine() is called from createOrthogonal().
  ISmaccStateMachine * stateMachine_ = nullptr;

  std::vector<std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>>> clientBehaviors_;
  friend class ISmaccStateMachine;

  // Guards clientBehaviors_. It is a leaf lock: it is never held while behavior code runs or
  // while any other lock is taken. runtimeConfigure(), onEntry() and onExit() copy the current
  // state's behavior list under it and then call the behaviors without it, so a behavior that
  // takes the state machine's m_mutex_ (requiresComponent, createSignalConnection) does not
  // create an orthogonal-then-state-machine lock order; the exit path already holds m_mutex_
  // when it locks this mutex. CbSequence adds behaviors through addClientBehavior from its own
  // thread, which is what the lock is for.
  std::mutex mutex_;

  std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>> currentClientBehaviors();
};

}  // namespace smacc2
