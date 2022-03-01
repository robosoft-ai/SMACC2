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

#include <nav2z_client/client_behaviors/cb_wait_nav2_nodes.hpp>
#include <nav2z_client/common.hpp>

namespace cl_nav2z
{
CbWaitNav2Nodes::CbWaitNav2Nodes(std::vector<Nav2Nodes> waitNodes)
: CbWaitNav2Nodes("/bond", waitNodes)
{
}

CbWaitNav2Nodes::CbWaitNav2Nodes(std::string topic, std::vector<Nav2Nodes> waitNodes)
: topicname_(topic), waitNodes_(waitNodes)
{
}

void CbWaitNav2Nodes::onMessageReceived(const bond::msg::Status & msg)
{
  auto value = fromString(msg.id);
  bool updated = false;
  // RCLCPP_INFO(getLogger(), "[CbWaitNav2Nodes] received '%s'", msg.id.c_str());

  if (receivedAliveMsg_.count(value) && !receivedAliveMsg_[value])
  {
    RCLCPP_INFO(getLogger(), "[CbWaitNav2Nodes] '%s' alive received", msg.id.c_str());
    receivedAliveMsg_[value] = true;
    updated = true;
  }

  if (updated)
  {
    bool success = true;
    std::stringstream ss;
    for (auto & pair : receivedAliveMsg_)
    {
      if (!pair.second)
      {
        success = false;
      }
      ss << "- " << toString(pair.first) << ": " << (pair.second ? "ALIVE" : "WAITING")
         << std::endl;
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[CbWaitNav2Nodes] waiting nodes status:" << std::endl
                                                             << ss.str().c_str());

    if (success)
    {
      RCLCPP_INFO(getLogger(), "[CbWaitNav2Nodes] success event");
      this->postSuccessEvent();
      sub_ = nullptr;
    }
    else
    {
      RCLCPP_INFO(getLogger(), "[CbWaitNav2Nodes] still missing nodes");
    }
  }
}

void CbWaitNav2Nodes::onEntry()
{
  std::stringstream ss;
  for (auto v : waitNodes_)
  {
    receivedAliveMsg_[v] = false;
    ss << "[CbWaitNav2Nodes] - " << toString(v) << ": waiting" << std::endl;
  }
  RCLCPP_INFO(getLogger(), ss.str().c_str());

  //rclcpp::SensorDataQoS qos;
  rclcpp::SubscriptionOptions sub_option;

  sub_ = getNode()->create_subscription<bond::msg::Status>(
    topicname_, 20, std::bind(&CbWaitNav2Nodes::onMessageReceived, this, std::placeholders::_1),
    sub_option);
}

std::string toString(Nav2Nodes value)
{
  switch (value)
  {
    case Nav2Nodes::PlannerServer:
      return "planner_server";
    case Nav2Nodes::ControllerServer:
      return "controller_server";
    case Nav2Nodes::RecoveriesServer:
      return "recoveries_server";
    case Nav2Nodes::BtNavigator:
      return "bt_navigator";
    case Nav2Nodes::MapServer:
      return "map_server";
    default:
      return "";
  }
}

Nav2Nodes fromString(std::string id)
{
  if (id == "planner_server")
    return Nav2Nodes::PlannerServer;
  else if (id == "controller_server")
    return Nav2Nodes::ControllerServer;
  else if (id == "recoveries_server")
    return Nav2Nodes::RecoveriesServer;
  else if (id == "bt_navigator")
    return Nav2Nodes::BtNavigator;
  else if (id == "map_server")
    return Nav2Nodes::MapServer;
  else
    return Nav2Nodes::None;
}

}  // namespace cl_nav2z
