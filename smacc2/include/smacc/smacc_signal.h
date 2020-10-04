/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <boost/signals2/signal.hpp>
#include <boost/any.hpp>

namespace smacc
{
using namespace boost;
using namespace boost::signals2;
using namespace boost::signals2::detail;

template <typename Signature,
          typename Combiner = optional_last_value<typename boost::function_traits<Signature>::result_type>,
          typename Group = int,
          typename GroupCompare = std::less<Group>,
          typename SlotFunction = function<Signature>,
          typename ExtendedSlotFunction = typename extended_signature<function_traits<Signature>::arity, Signature>::function_type,
          typename Mutex = boost::signals2::mutex>
class SmaccSignal : public boost::signals2::signal<Signature, Combiner, Group, GroupCompare, SlotFunction, ExtendedSlotFunction, Mutex>
{
};
} // namespace smacc