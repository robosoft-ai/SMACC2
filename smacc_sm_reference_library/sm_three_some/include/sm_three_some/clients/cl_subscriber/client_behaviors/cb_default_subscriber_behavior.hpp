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
#include <sm_three_some/clients/cl_subscriber/cl_subscriber.hpp>

namespace sm_three_some
{
namespace cl_subscriber
{
class CbDefaultSubscriberBehavior : public smacc2::SmaccClientBehavior
{
public:
  typedef std_msgs::msg::UInt16 TMessageType;
  //-------------------------------------------------------------------------------
  void onEntry() {}
};
}  // namespace cl_subscriber
}  // namespace sm_three_some
