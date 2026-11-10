// Copyright 2025 Robosoft Inc.
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
#include <cl_nav2z/components/bond_listener/cp_bond_listener.hpp>

namespace cl_nav2z
{
CpBondListener::CpBondListener(std::string topicName) : topicName_(topicName) {}

void CpBondListener::onInitialize()
{
  RCLCPP_INFO_STREAM(getLogger(), "[CpBondListener] subscribing to " << topicName_);
  sub_ = getNode()->create_subscription<bond::msg::Status>(
    topicName_, 20, [this](const bond::msg::Status::SharedPtr msg) { onBondMessage_(*msg); });
}
}  // namespace cl_nav2z
