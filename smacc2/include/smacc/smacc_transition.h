/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/introspection/introspection.h>
#include <smacc/introspection/state_traits.h>
namespace smacc
{

//////////////////////////////////////////////////////////////////////////////
template <class Event,
          class Destination,
          typename Tag,
          class TransitionContext,
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
    static boost::statechart::result react_without_action(State &stt)
    {
      RCLCPP_DEBUG(stt.getNode()->get_logger(),"[Smacc Transition] REACT WITHOUT ACTION");
      typedef smacc::Transition<Event, Destination, Tag, TransitionContext, pTransitionAction> Transtype;
      TRANSITION_TAG mock;
      specificNamedOnExit(stt, mock);

      stt.template notifyTransition<Transtype>();
      return stt.template transit<Destination>();
    }

    static boost::statechart::result react_with_action(State &stt, const Event &evt)
    {
      RCLCPP_DEBUG(stt.getNode()->get_logger(),"[Smacc Transition] REACT WITH ACTION AND EVENT");
      typedef smacc::Transition<Event, Destination, Tag, TransitionContext, pTransitionAction> Transtype;
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
      State &stt, const EventBase &evt, const IdType &eventType)
  {
    typedef boost::statechart::detail::reaction_dispatcher<
        reactions<State>, State, EventBase, Event, TransitionContext, IdType>
        dispatcher;
    return dispatcher::react(stt, evt, eventType);
  }
};
} // namespace smacc

