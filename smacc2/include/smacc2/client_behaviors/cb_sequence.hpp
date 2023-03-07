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
class CbSequence : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbSequence();

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

  // template <typename TOrthogonal, typename TSourceObject>
  // void onOrthogonalAllocation()
  // {
  //   smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  // }

  void onExit() override { sequenceNodes_.clear(); }

  template <typename TOrthogonal, typename TBehavior, typename... Args>
  CbSequence * then(Args &&... args)
  {
    std::function<std::shared_ptr<smacc2::SmaccAsyncClientBehavior>()> delayedCBFactoryFn =
      [this, args...]()
    {
      RCLCPP_INFO(
        getLogger(), "[CbSequence] then creating new sub behavior %s ",
        demangleSymbol<TBehavior>().c_str());
      auto createdBh = std::shared_ptr<TBehavior>(new TBehavior(args...));

      this->getCurrentState()->getOrthogonal<TOrthogonal>()->addClientBehavior(createdBh);
      createdBh->template onOrthogonalAllocation<TOrthogonal, TBehavior>();

      return createdBh;
    };

    sequenceNodes_.push_back(delayedCBFactoryFn);

    return this;
  }

private:
  void onSubNodeSuccess();
  void onSubNodeAbort();
  void recursiveConsumeNext();

  std::list<std::function<std::shared_ptr<smacc2::SmaccAsyncClientBehavior>()>> sequenceNodes_;
  boost::signals2::connection conn_;
  boost::signals2::connection conn2_;

  std::shared_ptr<smacc2::SmaccAsyncClientBehavior> bh_;
  std::atomic<int> consume_{0};
};
}  // namespace client_behaviors
}  // namespace smacc2
