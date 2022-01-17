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

#include <boost/statechart/transition.hpp>

namespace smacc2
{
class ISmaccState;
class ISmaccStateMachine;
class ISmaccClient;
class ISmaccUpdatable;
class ISmaccComponent;
class ISmaccClientBehavior;
class SmaccClientBehavior;
class SmaccAsyncClientBehavior;
class SignalDetector;

class StateReactor;
class SmaccEventGenerator;

namespace client_bases
{
class ISmaccActionClient;
class ISmaccSubscriber;
}  // namespace client_bases

namespace introspection
{
class SmaccStateMachineInfo;
class SmaccStateInfo;
class StateReactorHandler;
class SmaccStateReactorInfo;
class SmaccEventGeneratorInfo;
class TypeInfo;
}  // namespace introspection

// ----TAGS FOR TRANSITIONS -----

namespace default_transition_tags
{
// you can also use these other labels in order to have
// a better code readability and also to improve the visual representation
// in the viewer
struct DEFAULT
{
};

struct ABORT
{
};

struct SUCCESS
{
};

struct CANCEL
{
};

/*
struct PREEMPT
{
};


struct REJECT
{
};*/

struct CONTINUELOOP
{
};
struct ENDLOOP
{
};

struct default_transition_name : SUCCESS
{
};
}  // namespace default_transition_tags

template <
  class Event, class Destination, typename Tag = default_transition_tags::default_transition_name,
  class TransitionContext = boost::statechart::detail::no_context<Event>,
  void (TransitionContext::*pTransitionAction)(const Event &) =
    &boost::statechart::detail::no_context<Event>::no_function>
class Transition;

}  // namespace smacc2
