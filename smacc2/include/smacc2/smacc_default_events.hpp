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

#pragma once

#include <smacc2/smacc_types.hpp>

#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>

namespace smacc2
{
namespace default_events
{
using namespace smacc2::introspection;
using namespace smacc2::default_transition_tags;

//-------------- ACTION EVENTS --------------------------------------------------------
template <typename ActionFeedback, typename TOrthogonal>
struct EvActionFeedback : sc::event<EvActionFeedback<ActionFeedback, TOrthogonal>>
{
  smacc2::client_bases::ISmaccActionClient * client;
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
}  // namespace smacc2
