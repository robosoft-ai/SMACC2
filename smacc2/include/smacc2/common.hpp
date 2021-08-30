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

// boost statechart
#include <boost/statechart/asynchronous_state_machine.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>

// other boost includes
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/config.hpp>
#include <boost/function.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/mpl/list.hpp>
#include <boost/signals2.hpp>

#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc_fifo_scheduler.hpp>
#include <smacc2/smacc_types.hpp>

typedef boost::statechart::processor_container<
  boost::statechart::fifo_scheduler<>, boost::function0<void>,
  std::allocator<boost::statechart::none>>::processor_context my_context;
namespace smacc2
{
namespace utils
{
// demangles the type name to be used as a ros node name
std::string cleanShortTypeName(const std::type_info & tinfo);

template <typename T>
std::string cleanShortTypeName()
{
  return cleanShortTypeName(typeid(T));
}
}  // namespace utils

enum class SMRunMode
{
  DEBUG,
  RELEASE
};

template <typename StateMachineType>
void run();
}  // namespace smacc2

#include <smacc2/smacc_default_events.hpp>
#include <smacc2/smacc_transition.hpp>
