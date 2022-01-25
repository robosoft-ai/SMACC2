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

#include <smacc2/client_base_components/cp_topic_publisher.hpp>
#include <smacc2/client_base_components/cp_topic_subscriber.hpp>

//#include <autoware_auto_msgs//>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <smacc2/smacc_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

namespace sm_autoware_avp
{
namespace clients
{

  using namespace std::chrono_literals;
template <typename TSource, typename TObject>
struct EvAutoLocalized : sc::event<EvAutoLocalized<TSource, TObject>>
{
  geometry_msgs::msg::PoseWithCovarianceStamped location;
};

template <typename TSource, typename TObject>
struct EvGoalReached : sc::event<EvGoalReached<TSource, TObject>>
{
};

class ClAutoware : public smacc2::ISmaccClient
{
public:
  smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseWithCovarianceStamped> * cppubLocation_;

  smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped> * cpppubGoalPose_;

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> lastPose_;
  std::optional<geometry_msgs::msg::PoseStamped> lastGoal_;

  double goalToleranceMeters_ = 3.5;  //m

  std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped &)> postEvAutoLocalized_;
  std::function<void()> postEvGoalReached_;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postEvAutoLocalized_ = [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
    {
      auto ev = new EvAutoLocalized<TSourceObject, TOrthogonal>();
      ev->location = msg;
      this->postEvent(ev);
    };

    this->postEvGoalReached_ = [this]()
    {
      this->postEvent<EvGoalReached<TSourceObject, TOrthogonal>>();
    };
  }

  void onInitialize() override
  {
    /*
      auto cppubLocation = client->createNamedComponent<
      smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped>>(
      "initialPoseEstimation", "/localization/initialpose");

    auto cpppubGoalPose = client->createNamedComponent<
      smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped>>(
      "goalPose", "planning/goal_pose");
    */

    cppubLocation_ =
      this->getComponent<smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseWithCovarianceStamped>>(
        "initialPoseEstimation");

    cpppubGoalPose_ =
      this->getComponent<smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped>>(
        "goalPose");

    // auto subscriberNdtPose = client->createNamedComponent<
    //   smacc2::components::CpTopicSubscriber<geometry_msgs::msg::PoseWithCovarianceStamped>>(
    //   "ndtPose", "/localization/ndt_pose");

    auto subscriberNdtPose = this->getComponent<
      smacc2::components::CpTopicSubscriber<geometry_msgs::msg::PoseWithCovarianceStamped>>(
      "ndtPose");

    subscriberNdtPose->onMessageReceived(&ClAutoware::onNdtPoseReceived, this);
  }

  bool checkGoalReached()
  {
    if (lastGoal_ && lastPose_)
    {
      auto & pos = this->lastPose_->pose.pose.position;
      auto & goalpos = this->lastGoal_->pose.position;

      double dx = pos.x - goalpos.x;
      double dy = pos.y - goalpos.y;
      double dz = pos.z - goalpos.z;

      double dist = sqrt(dx * dx + dy * dy + dz * dz);

      RCLCPP_INFO_STREAM_THROTTLE(getLogger(), *(getNode()->get_clock()),2000,  "[" << getName() << "] goal linear error: " << dist);

      if (dist < goalToleranceMeters_)
      {
        return true;
      }
    }

    return false;
  }

  void onNdtPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    lastPose_ = msg;
    this->postEvAutoLocalized_(msg);

    if(checkGoalReached())
    {
      this->postEvGoalReached_();
    }
  }

  void publishGoalPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    lastGoal_ = msg;
    this->cpppubGoalPose_->publish(msg);
  }

  void publishInitialPoseEstimation(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    this->cppubLocation_->publish(msg);
  }
};
}  // namespace clients
}  // namespace sm_autoware_avp
