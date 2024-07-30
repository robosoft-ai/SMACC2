
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

#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit2z_client/common.hpp>
#include <moveit2z_client/cl_moveit2z.hpp>

#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <condition_variable>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace cl_moveit2z
{
class CbMoveSynchronizedLinkGoals : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbMoveSynchronizedLinkGoals(const std::vector<geometry_msgs::msg::PoseStamped>& poses, const std::vector<std::string>& names)
  {

  }

  void onEntry() override
  {
    ClMoveit2z * movegroupClient_;
    requiresClient(movegroupClient_);

    //auto group_name = movegroupClient_->getOptions().group_name_;
    std::string ns =  movegroupClient_->getOptions().move_group_namespace_;

    //auto topicname = "/joint_state_broadcaster_"UR5PREFIX"/joint_states";

    std::string topicname = "/joint_states";
    if (!ns.empty())
    {
        topicname = ns + "/" + topicname;
    }

    //rclcpp::SensorDataQoS qos;
    rclcpp::SubscriptionOptions sub_option;

    sub_ = getNode()->create_subscription<sensor_msgs::msg::JointState>(
        topicname, 20, std::bind(&CbMoveSynchronizedLinkGoals::onMessageReceived, this, std::placeholders::_1),
        sub_option);

        mutex.lock();
    }

private:
  void onMessageReceived(const sensor_msgs::msg::JointState & msg)
  {
      postSuccessEvent();
      mutex.unlock();
      sub_ = nullptr; // unsuscribe
  }

  bool triggered = false;
  std::mutex mutex;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};
}
