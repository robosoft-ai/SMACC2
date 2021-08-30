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

#include <smacc2/client_bases/smacc_publisher_client.hpp>

namespace cl_ros_publisher
{
class ClRosPublisher : public smacc2::client_bases::SmaccPublisherClient
{
public:
  ClRosPublisher();
  ~ClRosPublisher();

  template <typename MessageType>
  void configure(std::string topicName)
  {
    SmaccPublisherClient::configure<MessageType>(topicName);
  }

  template <typename MessageType>
  void publish(const MessageType & msg)
  {
    SmaccPublisherClient::publish(msg);
  }
};
}  // namespace cl_ros_publisher
