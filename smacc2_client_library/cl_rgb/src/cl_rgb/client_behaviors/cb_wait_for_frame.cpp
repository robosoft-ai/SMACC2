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

#include <cl_rgb/client_behaviors/cb_wait_for_frame.hpp>

namespace cl_rgb
{

CbWaitForFrame::CbWaitForFrame(const WaitForFrameParams & params)
: CbRgbClientBehaviorBase(params.camera), params_(params)
{
}

void CbWaitForFrame::wireCompletionSignals()
{
  if (camera_ == nullptr)
  {
    return;
  }
  const FramePtr latest = camera_->latestFrame();
  startSequence_ = latest ? latest->sequence : 0;
  this->getStateMachine()->createSignalConnection(
    camera_->onFrame_, &CbWaitForFrame::onFrame, this);
}

void CbWaitForFrame::onEntry()
{
  if (camera_ == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "CbWaitForFrame: no camera named '%s'", params_.camera.c_str());
    this->postRgbFailure();
    return;
  }
  RCLCPP_INFO(
    getLogger(), "CbWaitForFrame: waiting for %u new image(s) of '%s'", params_.frames,
    params_.camera.c_str());
}

void CbWaitForFrame::onFrame(const FramePtr & frame)
{
  if (frame->sequence >= startSequence_.load() + params_.frames)
  {
    this->postRgbSuccess();
  }
}

}  // namespace cl_rgb
