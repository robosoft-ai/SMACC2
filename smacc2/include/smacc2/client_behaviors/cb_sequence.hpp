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

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
class CbSequenceNode : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbSequenceNode();

  // template <typename TOrthogonal, typename TBehavior>
  // static void configure_orthogonal_runtime(
  //   std::function<void(TBehavior & bh, MostDerived &)> initializationFunction)
  // {
  //   configure_orthogonal_internal<TOrthogonal, TBehavior>([=](ISmaccState * state) {
  //     // auto bh = std::make_shared<TBehavior>(args...);
  //     // auto bh = state->configure<TOrthogonal, TBehavior>();
  //     initializationFunction(*bh, *(static_cast<MostDerived *>(state)));
  //   });
  // }

  void onEntry() override;

  template <typename TBehavior, typename... Args>
  CbSequenceNode * then(Args &&... args)
  {
    sequenceNodes_.push_back(new TBehavior(args...));
    return this;
  }

private:
  std::vector<smacc2::SmaccAsyncClientBehavior *> sequenceNodes_;
};
}  // namespace client_behaviors
}  // namespace smacc2
