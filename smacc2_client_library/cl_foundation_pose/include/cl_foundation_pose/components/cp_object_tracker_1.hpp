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

#pragma once
#include <cl_foundation_pose/components/tracker_utils.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/component.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace cl_foundation_pose
{

using namespace smacc2::client_core_components;

class CpObjectTracker1 : public smacc2::ISmaccComponent
{
public:
  // Declare the Component's default constructor.
  void onInitialize()
  {
    // Gain access to the foundationpose subscriber component.
    requiresComponent(fondationPoseTopic_);

    // Then hook each received message to store it into our little map/database.
    fondationPoseTopic_->onMessageReceived(&CpObjectTracker1::onDetection3DArrayReceived, this);
  }

  void onDetection3DArrayReceived(const vision_msgs::msg::Detection3DArray & msg)
  {
    RCLCPP_INFO(getLogger(), "Received %ld detections", msg.detections.size());

    for (auto & detection : msg.detections)
    {
      auto previouslyExistingObjectEntry = detectedObjects.find(detection.id);

      // if we have seen this object before...
      if (previouslyExistingObjectEntry != detectedObjects.end())
      {
        auto & previouslyExistingObject = previouslyExistingObjectEntry->second;
        previouslyExistingObject.msg = detection;
      }
      else
      {
        DetectedObject detectedObject;
        detectedObject.msg = detection;
        detectedObjects[detection.id] = detectedObject;
      }

      this->postEvent<EvObjectDetected>();
    }
  }

  std::optional<geometry_msgs::msg::PoseStamped> getPose(const std::string & object_id)
  {
    auto object = detectedObjects.find(object_id);
    if (object != detectedObjects.end())
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = object->second.msg.header;
      pose.pose = object->second.msg.results[0].pose.pose;
      return pose;
    }
    return std::nullopt;
  }

private:
  // Declare the subscriber component.
  CpTopicSubscriber<vision_msgs::msg::Detection3DArray> * fondationPoseTopic_;

  // Declare a data structure to store the detected objects.
  std::map<std::string, DetectedObject> detectedObjects;
};

}  // namespace cl_foundation_pose
