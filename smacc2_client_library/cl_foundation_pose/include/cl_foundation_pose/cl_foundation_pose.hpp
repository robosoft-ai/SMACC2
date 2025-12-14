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

#include <smacc2/smacc_client.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <smacc2/client_core_components/cp_topic_subscriber.hpp>

namespace cl_foundation_pose
{
using namespace smacc2::client_core_components;

class ClFoundationPose : public smacc2::ISmaccClient
{
public:
  ClFoundationPose() {}

  void onInitialize() override
  {
    //auto subcomponent = this->createComponent<CpTopicSubscriber<vision_msgs::msg::Detection3DArray>>("/detection3d_array");
  }
};

}  // namespace cl_foundation_pose
