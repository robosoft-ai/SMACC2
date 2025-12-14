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

#include <smacc2/component.hpp>

#include <algorithm>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace cl_isaac_apriltag
{

/**
 * @brief Component that manages mission-level AprilTag state.
 *
 * This component tracks which AprilTags have been visited and which tag
 * is currently selected for navigation. It provides thread-safe access
 * to mission state, separating mission logic from detection logic.
 */
class CpAprilTagMissionState : public smacc2::ISmaccComponent
{
public:
  CpAprilTagMissionState() = default;

  virtual ~CpAprilTagMissionState() = default;

  void onInitialize() override { RCLCPP_INFO(getLogger(), "[CpAprilTagMissionState] Initialized"); }

  /**
   * @brief Check if a tag has been visited.
   * @param tag_frame_id The tag frame ID (e.g., "36h11:5")
   * @return true if the tag has been marked as visited
   */
  bool isTagVisited(const std::string & tag_frame_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::find(visitedWorkingAreas_.begin(), visitedWorkingAreas_.end(), tag_frame_id) !=
           visitedWorkingAreas_.end();
  }

  /**
   * @brief Mark a tag as visited.
   * @param tag_frame_id The tag frame ID to mark as visited
   */
  void markTagVisited(const std::string & tag_frame_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      std::find(visitedWorkingAreas_.begin(), visitedWorkingAreas_.end(), tag_frame_id) ==
      visitedWorkingAreas_.end())
    {
      visitedWorkingAreas_.push_back(tag_frame_id);
      RCLCPP_INFO_STREAM(
        getLogger(), "[CpAprilTagMissionState] Marked tag as visited: " << tag_frame_id);
    }
  }

  /**
   * @brief Clear all visited tags.
   */
  void clearVisitedTags()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    visitedWorkingAreas_.clear();
    RCLCPP_INFO(getLogger(), "[CpAprilTagMissionState] Cleared all visited tags");
  }

  /**
   * @brief Get a copy of all visited tag IDs.
   * @return Vector of visited tag frame IDs
   */
  std::vector<std::string> getVisitedTags() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return visitedWorkingAreas_;
  }

  /**
   * @brief Select a tag for navigation.
   * @param tag_frame_id The tag frame ID to select
   */
  void selectTag(const std::string & tag_frame_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    selectedVisitTagId_ = tag_frame_id;
    RCLCPP_INFO_STREAM(
      getLogger(), "[CpAprilTagMissionState] Selected tag for visit: " << tag_frame_id);
  }

  /**
   * @brief Get the currently selected tag.
   * @return Optional containing the selected tag frame ID, or nullopt if none selected
   */
  std::optional<std::string> getSelectedTag() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return selectedVisitTagId_;
  }

  /**
   * @brief Clear the currently selected tag.
   */
  void clearSelectedTag()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    selectedVisitTagId_.reset();
    RCLCPP_INFO(getLogger(), "[CpAprilTagMissionState] Cleared selected tag");
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::string> visitedWorkingAreas_;
  std::optional<std::string> selectedVisitTagId_;
};

}  // namespace cl_isaac_apriltag
