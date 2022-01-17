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

#include <boost/any.hpp>
#include <boost/signals2/signal.hpp>

namespace smacc2
{
using namespace boost;
using namespace boost::signals2;
using namespace boost::signals2::detail;

template <
  typename Signature,
  typename Combiner = optional_last_value<typename boost::function_traits<Signature>::result_type>,
  typename Group = int, typename GroupCompare = std::less<Group>,
  typename SlotFunction = function<Signature>,
  typename ExtendedSlotFunction =
    typename extended_signature<function_traits<Signature>::arity, Signature>::function_type,
  typename Mutex = boost::signals2::mutex>
class SmaccSignal
: public boost::signals2::signal<
    Signature, Combiner, Group, GroupCompare, SlotFunction, ExtendedSlotFunction, Mutex>
{
};
}  // namespace smacc2
