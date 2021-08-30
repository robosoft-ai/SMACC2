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

#include <algorithm>
#include <boost/statechart/event.hpp>
#include <functional>
#include <map>
#include <memory>
#include <smacc2/introspection/introspection.hpp>
#include <vector>

namespace smacc2
{
class ISmaccState;
class ISMaccStateMachine;

namespace state_reactors
{
struct EmptyObjectTag
{
};
}  // namespace state_reactors

class StateReactor
{
public:
  ISmaccState * ownerState;
  std::function<void()> postEventFn;
  std::vector<const std::type_info *> eventTypes;
  std::map<const std::type_info *, std::function<void(void *)>> eventCallbacks_;

  StateReactor();
  virtual ~StateReactor();

  virtual void onInitialized();

  virtual void onEntry();
  virtual void onExit();

  virtual void onEventNotified(const std::type_info * eventType);

  template <typename EventType>
  void postEvent(const EventType & ev);

  template <typename EventType>
  void postEvent();

  // type based event callback
  template <typename T, typename TClass>
  void createEventCallback(void (TClass::*callback)(T *), TClass * object);

  // type based event callback
  template <typename T>
  void createEventCallback(std::function<void(T *)> callback);

  void update();

  //must returns true when the output event is triggered
  virtual bool triggers() = 0;

  template <typename TEv>
  void addInputEvent();

  template <typename TEv>
  void setOutputEvent();

  //TDerived
  void initialize(ISmaccState * ownerState);

  rclcpp::Node::SharedPtr getNode();

  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }

private:
  friend ISmaccStateMachine;

  template <typename TEvent>
  void notifyEvent(TEvent * ev)
  {
    //the state machine uses this method to notify this state reactor some event happened.
    auto tid = &(typeid(TEvent));
    if (std::find(eventTypes.begin(), eventTypes.end(), tid) != eventTypes.end())
    {
      this->onEventNotified(tid);
      this->update();

      if (eventCallbacks_.count(tid))
      {
        eventCallbacks_[tid]((void *)ev);
      }
    }
  }
};

}  // namespace smacc2
