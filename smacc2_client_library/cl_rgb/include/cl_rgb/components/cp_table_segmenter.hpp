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

#include <cl_rgb/table_survey.hpp>

#include <opencv2/core.hpp>
#include <smacc2/smacc.hpp>

#include <mutex>

namespace cl_rgb
{

// Separates what lies on the table from the table. With a reference image of the empty scene
// it compares pixel by pixel; without one it takes the table's colour from the image itself,
// which holds while the table fills most of its region and the objects differ from it in colour.
class CpTableSegmenter : public smacc2::ISmaccComponent
{
public:
  explicit CpTableSegmenter(const SegmenterParams & params);
  ~CpTableSegmenter() override;

  void onInitialize() override;

  void setParams(const SegmenterParams & params);
  SegmenterParams params() const;

  // the empty scene as the camera sees it; an empty image removes the reference
  void setReference(const cv::Mat & bgr);
  bool hasReference() const;

  // 255 where the image shows something on the table, within tableMask
  // `shadow`, when given, receives the pixels taken for shadow (see segmentAgainstReference)
  cv::Mat foreground(
    const cv::Mat & bgr, const cv::Mat & tableMask, cv::Mat * shadow = nullptr) const;

  // 255 where the reference image itself shows something that is not table, within tableMask:
  // what stands in the empty scene and may hide part of an object. Empty without a reference.
  cv::Mat fixtures(const cv::Mat & tableMask) const;

private:
  mutable std::mutex mutex_;
  SegmenterParams params_;
  cv::Mat reference_;
};

}  // namespace cl_rgb
