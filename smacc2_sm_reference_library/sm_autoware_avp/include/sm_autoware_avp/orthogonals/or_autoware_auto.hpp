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

#include <chrono>

#include "sm_autoware_avp/clients/autoware_client/cl_autoware.hpp"
#include "smacc2/smacc.hpp"

namespace sm_autoware_avp
{

class OrAutowareAuto : public smacc2::Orthogonal<OrAutowareAuto>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<sm_autoware_avp::clients::ClAutoware>();

    client->createNamedComponent<
      smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseWithCovarianceStamped>>(
      "initialPoseEstimation", "/localization/initialpose");

    client->createNamedComponent<
      smacc2::components::CpTopicPublisher<geometry_msgs::msg::PoseStamped>>(
      "goalPose", "planning/goal_pose");

    client->createNamedComponent<
      smacc2::components::CpTopicSubscriber<geometry_msgs::msg::PoseWithCovarianceStamped>>(
      "ndtPose", "/localization/ndt_pose");
  }
};
}  // namespace sm_autoware_avp
