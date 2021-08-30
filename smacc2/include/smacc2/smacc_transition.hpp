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

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/introspection/state_traits.hpp>
namespace smacc2
{
//////////////////////////////////////////////////////////////////////////////
template <
  class Event, class Destination, typename Tag, class TransitionContext,
  void (TransitionContext::*pTransitionAction)(const Event &)>
class Transition
{
public:
  typedef Tag TRANSITION_TAG;

private:
  //////////////////////////////////////////////////////////////////////////
  template <class State>
  struct reactions
  {
    static boost::statechart::result react_without_action(State & stt)
    {
      RCLCPP_DEBUG(stt.getLogger(), "[Smacc Transition] REACT WITHOUT ACTION");
      typedef smacc2::Transition<Event, Destination, Tag, TransitionContext, pTransitionAction>
        Transtype;
      TRANSITION_TAG mock;
      specificNamedOnExit(stt, mock);

      stt.template notifyTransition<Transtype>();
      return stt.template transit<Destination>();
    }

    static boost::statechart::result react_with_action(State & stt, const Event & evt)
    {
      RCLCPP_DEBUG(stt.getLogger(), "[Smacc Transition] REACT WITH ACTION AND EVENT");
      typedef smacc2::Transition<Event, Destination, Tag, TransitionContext, pTransitionAction>
        Transtype;
      TRANSITION_TAG mock;
      specificNamedOnExit(stt, mock);
      stt.template notifyTransition<Transtype>();
      return stt.template transit<Destination>(pTransitionAction, evt);
    }
  };

public:
  //////////////////////////////////////////////////////////////////////////
  // The following declarations should be private.
  // They are only public because many compilers lack template friends.
  //////////////////////////////////////////////////////////////////////////
  template <class State, class EventBase, class IdType>
  static boost::statechart::detail::reaction_result react(
    State & stt, const EventBase & evt, const IdType & eventType)
  {
    typedef boost::statechart::detail::reaction_dispatcher<
      reactions<State>, State, EventBase, Event, TransitionContext, IdType>
      dispatcher;
    return dispatcher::react(stt, evt, eventType);
  }
};
}  // namespace smacc2
