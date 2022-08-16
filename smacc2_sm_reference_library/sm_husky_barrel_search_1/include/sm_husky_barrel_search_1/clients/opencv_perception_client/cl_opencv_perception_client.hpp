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

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sm_husky_barrel_search_1/msg/detected_objects.hpp>

namespace sm_husky_barrel_search_1
{
namespace cl_opencv_perception
{
struct EvDetectedBarrelColor : sc::event<EvDetectedBarrelColor>
{
};

template <typename AsyncCB, typename Orthogonal>
struct EvEnemyDetected : sc::event<EvEnemyDetected<AsyncCB, Orthogonal>>
{
};

template <typename AsyncCB, typename Orthogonal>
struct EvEnemyClusterDetected : sc::event<EvEnemyClusterDetected<AsyncCB, Orthogonal>>
{
};

class ClOpenCVPerception
  : public smacc2::client_bases::SmaccSubscriberClient<sm_husky_barrel_search_1::msg::DetectedObjects>
{
public:
  ClOpenCVPerception(std::string topicname = "/detected_objects")
    : smacc2::client_bases::SmaccSubscriberClient<sm_husky_barrel_search_1::msg::DetectedObjects>(topicname)
  {
  }

  virtual ~ClOpenCVPerception()
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc2::client_bases::SmaccSubscriberClient<sm_husky_barrel_search_1::msg::DetectedObjects>::onOrthogonalAllocation<
        TOrthogonal, TSourceObject>();

    this->postEvEnemyDetected = [this]() { this->postEvent<EvEnemyDetected<TSourceObject, TOrthogonal>>(); };

    this->postEvEnemyClusterDetected = [this]() {
      this->postEvent<EvEnemyClusterDetected<TSourceObject, TOrthogonal>>();
    };
  }

  void MessageCallback(const sm_husky_barrel_search_1::msg::DetectedObjects& detectedMsg)
  {
    int totalEnemies = 0;
    for (auto detectedObject : detectedMsg.detected_objects)
    {
      size_t found = detectedObject.find("enemy");

      if (found != std::string::npos)
      {
        this->postEvEnemyDetected();
        totalEnemies++;
      }
    }

    if (totalEnemies >= 3)
    {
      this->postEvEnemyClusterDetected();
    }
  }

  void onInitialize() override
  {
    smacc2::client_bases::SmaccSubscriberClient<sm_husky_barrel_search_1::msg::DetectedObjects>::onInitialize();
    this->onMessageReceived(&ClOpenCVPerception::MessageCallback, this);
  }

private:
  std::function<void()> postEvEnemyDetected;
  std::function<void()> postEvEnemyClusterDetected;
};
}  // namespace cl_opencv_perception
}  // namespace sm_husky_barrel_search_1
