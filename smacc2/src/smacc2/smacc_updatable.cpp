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

#include <smacc2/smacc_updatable.hpp>

namespace smacc2
{
ISmaccUpdatable::ISmaccUpdatable() : lastUpdate_(0) {}

ISmaccUpdatable::ISmaccUpdatable(rclcpp::Duration duration)
: periodDuration_(duration), lastUpdate_(0)

{
}

void ISmaccUpdatable::setUpdatePeriod(rclcpp::Duration duration) { periodDuration_ = duration; }

void ISmaccUpdatable::executeUpdate(rclcpp::Node::SharedPtr node)
{
  bool update = true;
  if (periodDuration_)
  {
    auto now = node->now();
    auto elapsed = now - this->lastUpdate_;
    update = elapsed > *periodDuration_;
    if (update)
    {
      this->lastUpdate_ = now;
    }
  }

  if (update)
  {
    this->update();
  }
}
}  // namespace smacc2
