#pragma once

#include <smacc/smacc_types.h>

#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>

namespace smacc
{
namespace default_events
{
using namespace smacc::introspection;
using namespace smacc::default_transition_tags;

//-------------- ACTION EVENTS --------------------------------------------------------
template <typename ActionFeedback, typename TOrthogonal>
struct EvActionFeedback : sc::event<EvActionFeedback<ActionFeedback, TOrthogonal>>
{
  smacc::client_bases::ISmaccActionClient * client;
  ActionFeedback feedbackMessage;
  // boost::any feedbackMessage;
};

template <typename TSource, typename TOrthogonal>
struct EvActionResult : sc::event<EvActionResult<TSource, TOrthogonal>>
{
  typename TSource::WrappedResult resultMessage;
};

template <typename TSource, typename TOrthogonal>
struct EvActionSucceeded : sc::event<EvActionSucceeded<TSource, TOrthogonal>>
{
  typename TSource::WrappedResult resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag() { return demangledTypeName<SUCCESS>(); }

  static std::string getDefaultTransitionType() { return demangledTypeName<SUCCESS>(); }
};

template <typename TSource, typename TOrthogonal>
struct EvActionAborted : sc::event<EvActionAborted<TSource, TOrthogonal>>
{
  typename TSource::WrappedResult resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag() { return demangledTypeName<ABORT>(); }

  static std::string getDefaultTransitionType() { return demangledTypeName<ABORT>(); }
};

template <typename TSource, typename TOrthogonal>
struct EvActionCancelled : sc::event<EvActionCancelled<TSource, TOrthogonal>>
{
  typename TSource::WrappedResult resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag() { return demangledTypeName<CANCEL>(); }

  static std::string getDefaultTransitionType() { return demangledTypeName<CANCEL>(); }
};

//---------- CONTROL FLOW EVENTS ----------------------------------------------------------

template <typename StateType>
struct EvStateRequestFinish : sc::event<EvStateRequestFinish<StateType>>
{
};

template <typename StateType>
struct EvSequenceFinished : sc::event<EvSequenceFinished<StateType>>
{
};

template <typename TSource>
struct EvLoopContinue : sc::event<EvLoopContinue<TSource>>
{
  static std::string getDefaultTransitionTag() { return demangledTypeName<CONTINUELOOP>(); }

  static std::string getDefaultTransitionType() { return demangledTypeName<CONTINUELOOP>(); }
};

template <typename TSource>
struct EvLoopEnd : sc::event<EvLoopEnd<TSource>>
{
  static std::string getDefaultTransitionTag() { return demangledTypeName<ENDLOOP>(); }

  static std::string getDefaultTransitionType() { return demangledTypeName<ENDLOOP>(); }
};

//---------- CONTROL FLOW EVENTS ----------------------------------------------------------
template <typename TSource, typename TOrthogonal>
struct EvTopicInitialMessage : sc::event<EvTopicInitialMessage<TSource, TOrthogonal>>
{
  // typename EvTopicInitialMessage<SensorBehaviorType>::TMessageType msgData;
  static std::string getEventLabel()
  {
    auto typeinfo = TypeInfo::getTypeInfoFromType<typename TSource::TMessageType>();

    std::string label = typeinfo->getNonTemplatedTypeName();
    return label;
  }

  typename TSource::TMessageType msgData;
};

template <typename TSource, typename TOrthogonal>
struct EvTopicMessage : sc::event<EvTopicMessage<TSource, TOrthogonal>>
{
  static std::string getEventLabel()
  {
    auto typeinfo = TypeInfo::getTypeInfoFromType<typename TSource::TMessageType>();

    std::string label = typeinfo->getNonTemplatedTypeName();
    return label;
  }

  typename TSource::TMessageType msgData;
};
}  // namespace default_events
}  // namespace smacc