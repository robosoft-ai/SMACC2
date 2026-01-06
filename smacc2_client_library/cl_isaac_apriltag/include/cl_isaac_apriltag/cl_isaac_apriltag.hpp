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

#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client.hpp>

#include <cl_isaac_apriltag/components/cp_apriltag_mission_state.hpp>
#include <cl_isaac_apriltag/components/cp_apriltag_tracker.hpp>

#include <string>

namespace cl_isaac_apriltag
{

// Event: Unvisited AprilTag detected (templated for orthogonal/source)
template <typename AsyncCB, typename Orthogonal>
struct EvUnvisitedAprilTagDetected : sc::event<EvUnvisitedAprilTagDetected<AsyncCB, Orthogonal>>
{
};

using AprilTagDetectionArray = isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

/**
 * @brief Client for Isaac ROS AprilTag detection.
 *
 * This client follows the pure component-based architecture pattern.
 * It acts as an orchestrator that creates and wires components:
 *
 * - CpTopicSubscriber<AprilTagDetectionArray>: Subscribes to /tag_detections
 * - CpAprilTagTracker: Transforms detections to map frame, stores poses
 * - CpAprilTagMissionState: Manages visited/selected tag state
 *
 * Client behaviors should use requiresComponent() to access tracker and
 * mission state, not direct client fields.
 */
class ClIsaacApriltag : public smacc2::ISmaccClient
{
public:
  ClIsaacApriltag(std::string topic_name = "/tag_detections", std::string target_frame = "map")
  : topicName_(topic_name), targetFrame_(target_frame)
  {
  }

  virtual ~ClIsaacApriltag() {}

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // 1. Create core topic subscriber (from smacc2 framework)
    this->createComponent<
      smacc2::client_core_components::CpTopicSubscriber<AprilTagDetectionArray>, TOrthogonal,
      ClIsaacApriltag>(topicName_);

    // 2. Create tracker component (hooks to subscriber via requiresComponent)
    this->createComponent<CpAprilTagTracker, TOrthogonal, ClIsaacApriltag>(targetFrame_);

    // 3. Create mission state component
    this->createComponent<CpAprilTagMissionState, TOrthogonal, ClIsaacApriltag>();
  }

private:
  std::string topicName_;
  std::string targetFrame_;
};

}  // namespace cl_isaac_apriltag
