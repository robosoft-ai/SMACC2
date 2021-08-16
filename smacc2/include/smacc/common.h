/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
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

#include <smacc/introspection/introspection.h>
#include <smacc/smacc_fifo_scheduler.h>
#include <smacc/smacc_types.h>

typedef boost::statechart::processor_container<
  boost::statechart::fifo_scheduler<>, boost::function0<void>,
  std::allocator<boost::statechart::none>>::processor_context my_context;
namespace smacc
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
}  // namespace smacc

#include <smacc/smacc_default_events.h>
#include <smacc/smacc_transition.h>