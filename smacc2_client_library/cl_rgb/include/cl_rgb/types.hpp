// Copyright 2026 RobosoftAI Inc.
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

#include <cl_rgb/geometry.hpp>
#include <cl_rgb/table_survey.hpp>

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cl_rgb
{

// One image of one camera. Shared as a pointer to const: whoever holds it may keep it.
struct Frame
{
  cv::Mat image;  // BGR, 8 bit
  rclcpp::Time stamp;
  std::string frameId;    // the camera's optical frame
  uint64_t sequence = 0;  // counts the frames of this camera since start, from 1
};
using FramePtr = std::shared_ptr<const Frame>;

// What a survey of the table found in one frame.
struct Survey
{
  rclcpp::Time stamp;
  std::string camera;
  uint64_t sequence = 0;    // of the frame it was made from
  std::vector<Blob> blobs;  // largest first
  PinholeCamera model;      // the camera as it stood for that frame
};
using SurveyPtr = std::shared_ptr<const Survey>;

}  // namespace cl_rgb
