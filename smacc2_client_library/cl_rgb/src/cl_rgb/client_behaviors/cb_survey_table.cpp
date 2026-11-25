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

#include <cl_rgb/client_behaviors/cb_survey_table.hpp>

#include <chrono>
#include <thread>

namespace cl_rgb
{

CbSurveyTable::CbSurveyTable(const SurveyTableParams & params)
: CbRgbClientBehaviorBase(params.camera), params_(params)
{
}

void CbSurveyTable::onEntry()
{
  if (camera_ == nullptr || table_ == nullptr || segmenter_ == nullptr || survey_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(),
      "CbSurveyTable: a component is missing (camera '%s' %d, table %d, segmenter %d, survey %d)",
      params_.camera.c_str(), camera_ != nullptr, table_ != nullptr, segmenter_ != nullptr,
      survey_ != nullptr);
    this->postRgbFailure();
    return;
  }

  // The latest image, with the camera as it stood when the image was taken.
  FramePtr frame;
  std::optional<PinholeCamera> model;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(params_.waitS);
  while (!this->isShutdownRequested())
  {
    frame = camera_->latestFrame();
    if (frame)
    {
      model = camera_->model(*frame);
      if (model)
      {
        break;
      }
    }
    if (std::chrono::steady_clock::now() > deadline)
    {
      RCLCPP_ERROR(
        getLogger(),
        "CbSurveyTable: no usable image of '%s' after %.1f s (image %d, calibration and pose %d)",
        params_.camera.c_str(), params_.waitS, frame != nullptr, model.has_value());
      this->postRgbFailure();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!frame || !model)
  {
    this->postRgbFailure();
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  const cv::Mat region = table_->regionMask(*model, frame->image.size());
  // What a fixture covers and what was taken for shadow is not known to be empty table: an
  // object may go on there, so a fit must count those pixels neither for nor against it.
  cv::Mat unknown;
  const cv::Mat foreground = segmenter_->foreground(frame->image, region, &unknown);
  const cv::Mat fixtures = segmenter_->fixtures(region);
  if (fixtures.size() == unknown.size())
  {
    unknown |= fixtures;
  }
  auto survey = std::make_shared<Survey>();
  survey->stamp = frame->stamp;
  survey->camera = params_.camera;
  survey->model = *model;
  survey->sequence = frame->sequence;
  survey->blobs =
    findBlobs(frame->image, foreground, *model, table_->params(), params_.blobs, unknown);
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  RCLCPP_INFO(
    getLogger(), "CbSurveyTable: %zu blob(s) in image %lu of '%s' (%.1f ms)", survey->blobs.size(),
    static_cast<unsigned long>(frame->sequence), params_.camera.c_str(), ms);
  for (const auto & blob : survey->blobs)
  {
    RCLCPP_INFO(
      getLogger(),
      "CbSurveyTable:   blob %d at x %+.3f y %+.3f m, %.0f x %.0f mm, axis %+.0f deg, %d px, "
      "colour L %.0f a %+.0f b %+.0f",
      blob.id, blob.tablePoint[0], blob.tablePoint[1], blob.lengthM * 1000.0, blob.widthM * 1000.0,
      blob.axisRad * 180.0 / CV_PI, blob.areaPx, blob.meanLab[0], blob.meanLab[1], blob.meanLab[2]);
  }

  const bool enough = static_cast<int>(survey->blobs.size()) >= params_.minBlobs;
  survey_->store(survey);
  if (enough)
  {
    this->postRgbSuccess();
  }
  else
  {
    RCLCPP_WARN(
      getLogger(), "CbSurveyTable: %zu blob(s), %d needed - posting failure", survey->blobs.size(),
      params_.minBlobs);
    this->postRgbFailure();
  }
}

}  // namespace cl_rgb
