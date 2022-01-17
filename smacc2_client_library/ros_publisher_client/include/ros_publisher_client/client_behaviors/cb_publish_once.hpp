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
#include <ros_publisher_client/cl_ros_publisher.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace cl_ros_publisher
{
template <typename RosMsgType>
class CbPublishOnce : public smacc2::SmaccClientBehavior
{
private:
  std::function<void()> deferedPublishFn;
  ClRosPublisher * client_;

public:
  CbPublishOnce() : deferedPublishFn(nullptr) {}

  template <typename TMessage>
  CbPublishOnce(const TMessage & data)
  {
    this->setMessage(data);
  }

  template <typename TMessage>
  void setMessage(const TMessage & data)
  {
    deferedPublishFn = [=]() { client_->publish(data); };
  }

  void onEntry() override
  {
    this->requiresClient(client_);

    if (deferedPublishFn != nullptr) deferedPublishFn();
  }

  void onExit() override {}
};
}  // namespace cl_ros_publisher
