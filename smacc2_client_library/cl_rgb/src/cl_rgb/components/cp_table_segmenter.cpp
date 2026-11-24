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

#include <cl_rgb/components/cp_table_segmenter.hpp>

#include <opencv2/imgcodecs.hpp>

namespace cl_rgb
{

CpTableSegmenter::CpTableSegmenter(const SegmenterParams & params) : params_(params) {}

CpTableSegmenter::~CpTableSegmenter() {}

void CpTableSegmenter::onInitialize()
{
  const std::string path = this->params().referenceImagePath;
  if (path.empty())
  {
    return;
  }
  const cv::Mat reference = cv::imread(path, cv::IMREAD_COLOR);
  if (reference.empty())
  {
    RCLCPP_ERROR(
      getLogger(),
      "CpTableSegmenter: cannot read the reference image %s; segmenting by the table's own colour",
      path.c_str());
    return;
  }
  this->setReference(reference);
  RCLCPP_INFO(
    getLogger(), "CpTableSegmenter: reference image %s (%dx%d)", path.c_str(), reference.cols,
    reference.rows);
}

void CpTableSegmenter::setReference(const cv::Mat & bgr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  reference_ = bgr.clone();
}

bool CpTableSegmenter::hasReference() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !reference_.empty();
}

void CpTableSegmenter::setParams(const SegmenterParams & params)
{
  std::lock_guard<std::mutex> lock(mutex_);
  params_ = params;
}

SegmenterParams CpTableSegmenter::params() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return params_;
}

cv::Mat CpTableSegmenter::foreground(const cv::Mat & bgr, const cv::Mat & tableMask) const
{
  cv::Mat reference;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reference = reference_;  // shares the pixels; they are never written after setReference
  }
  if (!reference.empty() && reference.size() == bgr.size())
  {
    return segmentAgainstReference(bgr, reference, tableMask, this->params());
  }
  return segmentForeground(bgr, tableMask, this->params());
}

}  // namespace cl_rgb
