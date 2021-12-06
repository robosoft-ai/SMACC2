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
#include <sm_pubsub_1/clients/cl_subscriber/cl_subscriber.hpp>

namespace sm_pubsub_1
{
namespace cl_subscriber
{
template <typename TSource, typename TOrthogonal>
struct EvMyBehavior : sc::event<EvMyBehavior<TSource, TOrthogonal>>
{
};

class CbMySubscriberBehavior : public smacc2::SmaccClientBehavior
{
public:
  void onEntry()
  {
    ClSubscriber * client;
    this->requiresClient(client);

    client->onMessageReceived(&CbMySubscriberBehavior::onMessageReceived, this);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postMyEvent_ = [=] { this->postEvent<EvMyBehavior<TSourceObject, TOrthogonal>>(); };
  }

  void onMessageReceived(const std_msgs::msg::Float32 & msg)
  {
    if (msg.data > 30)
    {
      RCLCPP_INFO(
        getLogger(),
        "[CbMySubscriberBehavior] received message from topic with value > 30. "
        "Posting event!");
      this->postMyEvent_();
    }
  }

  std::function<void()> postMyEvent_;
};
}  // namespace cl_subscriber
}  // namespace sm_pubsub_1
