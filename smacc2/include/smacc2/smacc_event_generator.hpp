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

class SmaccEventGenerator
{
public:
  SmaccEventGenerator();
  virtual ~SmaccEventGenerator();

  template <typename TState, typename TSource>
  void onStateAllocation();

  virtual void onEntry();
  virtual void onExit();

  template <typename EventType>
  void postEvent(const EventType & ev);

  template <typename EventType>
  void postEvent();

  void initialize(ISmaccState * ownerState);
  virtual void onInitialized();

private:
  ISmaccState * ownerState_;
  friend ISmaccStateMachine;
};

}  // namespace smacc2
