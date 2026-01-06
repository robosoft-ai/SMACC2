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

#include <cl_isaac_apriltag/cl_isaac_apriltag.hpp>
#include <cl_isaac_apriltag/components/cp_apriltag_mission_state.hpp>
#include <cl_isaac_apriltag/components/cp_apriltag_tracker.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_isaac_apriltag
{

/**
 * @brief Behavior that detects unvisited AprilTags and posts events.
 *
 * This behavior uses requiresComponent() to access:
 * - CpAprilTagTracker: For detection signals
 * - CpAprilTagMissionState: For visited/selected tag management
 *
 * When an unvisited tag is detected, it selects the tag and posts
 * an EvUnvisitedAprilTagDetected event.
 */
class CbDetectAprilTag : public smacc2::SmaccAsyncClientBehavior
{
private:
  std::function<void()> postEvAprilTagDetected_;

  // Components accessed via requiresComponent (NOT direct client fields!)
  CpAprilTagTracker * tracker_ = nullptr;
  CpAprilTagMissionState * missionState_ = nullptr;

public:
  CbDetectAprilTag() {}

  virtual ~CbDetectAprilTag() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postEvAprilTagDetected_ = [=]()
    { this->postEvent<EvUnvisitedAprilTagDetected<TSourceObject, TOrthogonal>>(); };
  }

  virtual void onEntry() override
  {
    // Get components via requiresComponent pattern (NOT direct client access!)
    this->requiresComponent(tracker_, true);       // HARD requirement
    this->requiresComponent(missionState_, true);  // HARD requirement

    // Subscribe to tracker's detection signal
    tracker_->onAprilTagDetected(&CbDetectAprilTag::onAprilTagDetected, this);
  }

  void onAprilTagDetected(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr & msg)
  {
    for (auto & detection : msg->detections)
    {
      std::string apriltag_frame_id = detection.family + ":" + std::to_string(detection.id);

      // Check mission state via component (NOT direct client fields!)
      if (!missionState_->isTagVisited(apriltag_frame_id))
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[CbDetectAprilTag] new unvisited AprilTag detected: " << detection.id);

        // Select tag via component method (NOT direct client field assignment!)
        missionState_->selectTag(apriltag_frame_id);
        postEvAprilTagDetected_();
      }
      else
      {
        RCLCPP_INFO_STREAM_THROTTLE(
          getLogger(), *(getNode()->get_clock()), 2000,
          "[CbDetectAprilTag] Skipping AprilTag already detected: " << apriltag_frame_id);
      }
    }
  }
};

}  // namespace cl_isaac_apriltag
