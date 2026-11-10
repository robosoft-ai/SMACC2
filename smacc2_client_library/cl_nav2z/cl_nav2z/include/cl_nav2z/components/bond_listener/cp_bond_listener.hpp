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
#pragma once

#include <bond/msg/status.hpp>
#include <smacc2/smacc.hpp>

namespace cl_nav2z
{
// Owns the subscription to the nav2 bond heartbeat topic for the life of the state machine. Behaviors
// subscribe through onBondMessage, so the state-lifetime signal map finalizes their slot at dispose.
class CpBondListener : public smacc2::ISmaccComponent
{
public:
  CpBondListener(std::string topicName = "/bond");

  void onInitialize() override;

  template <typename T>
  smacc2::SmaccSignalConnection onBondMessage(
    void (T::*callback)(const bond::msg::Status &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onBondMessage_, callback, object);
  }

  const std::string & getTopicName() const { return topicName_; }

private:
  std::string topicName_;
  rclcpp::Subscription<bond::msg::Status>::SharedPtr sub_;
  smacc2::SmaccSignal<void(const bond::msg::Status &)> onBondMessage_;
};
}  // namespace cl_nav2z
