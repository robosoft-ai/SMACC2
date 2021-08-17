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

#include <smacc/client_bases/smacc_subscriber_client.h>
#include <smacc/smacc.h>

#include <std_msgs/msg/int16.hpp>

namespace sm_atomic_subscribers_performance_test
{
using namespace smacc::client_bases;

class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
  void onInitialize() override
  {
    this->createClient<SmaccSubscriberClient<std_msgs::msg::Int16>>("temperature");
  }
};
}  // namespace sm_atomic_subscribers_performance_test
