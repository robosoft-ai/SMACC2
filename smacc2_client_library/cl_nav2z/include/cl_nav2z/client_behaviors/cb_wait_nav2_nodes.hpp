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

#include <tf2_ros/buffer.h>

#include <bond/msg/status.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{
enum class Nav2Nodes
{
  PlannerServer,
  ControllerServer,
  RecoveriesServer,
  BtNavigator,
  MapServer,
  None
};

std::string toString(Nav2Nodes value);

Nav2Nodes fromString(std::string str);

class CbWaitNav2Nodes : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbWaitNav2Nodes(
    std::string topicname,
    std::vector<Nav2Nodes> waitNodes = {
      Nav2Nodes::PlannerServer, Nav2Nodes::ControllerServer, Nav2Nodes::BtNavigator});

  CbWaitNav2Nodes(
    std::vector<Nav2Nodes> waitNodes = {
      Nav2Nodes::PlannerServer, Nav2Nodes::ControllerServer, Nav2Nodes::BtNavigator});

  void onEntry() override;

private:
  void onMessageReceived(const bond::msg::Status & msg);

  std::string topicname_;

  rclcpp::Subscription<bond::msg::Status>::SharedPtr sub_;
  std::map<Nav2Nodes, bool> receivedAliveMsg_;

  std::vector<Nav2Nodes> waitNodes_;
};
}  // namespace cl_nav2z
