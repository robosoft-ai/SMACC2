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
#include <smacc2/component.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace cl_foundation_pose
{

struct DetectedObject
{
  vision_msgs::msg::Detection3D msg;

  std::optional<geometry_msgs::msg::PoseStamped> filtered_pose;

  std::vector<geometry_msgs::msg::PoseStamped> historicalPoses_;

  DetectedObject() { historicalPoses_.clear(); }
};

struct EvObjectDetected : sc::event<EvObjectDetected>
{
};

}  // namespace cl_foundation_pose
