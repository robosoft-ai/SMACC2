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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <cl_rgb/components/cp_camera.hpp>
#include <cl_rgb/components/cp_scene_survey.hpp>
#include <cl_rgb/components/cp_table_plane.hpp>
#include <cl_rgb/components/cp_table_segmenter.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_rgb
{

// Base for cl_rgb client behaviors. The components are resolved and the completion signals
// wired in onStateOrthogonalAllocation, on the state machine thread; onEntry, which runs on
// the behavior's own thread, only computes (see the async thread locking rule in CLAUDE.md).
//
// Derived behaviors:
// - override wireCompletionSignals() to connect a component signal, when they finish on one
// - finish through postRgbSuccess()/postRgbFailure(), which post once however often called
// - may arm a watchdog with setTimeout(); it is checked from update() and posts failure
class CbRgbClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior,
                                public smacc2::ISmaccUpdatable
{
public:
  explicit CbRgbClientBehaviorBase(const std::string & cameraName) : cameraName_(cameraName) {}

  ~CbRgbClientBehaviorBase() override {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresComponent(cameraName_, camera_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(table_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(segmenter_, smacc2::ComponentRequirement::SOFT);
    this->requiresComponent(survey_, smacc2::ComponentRequirement::SOFT);

    this->wireCompletionSignals();

    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  // wall time after which the behavior posts failure; never when not called
  void setTimeout(std::chrono::milliseconds timeout) { timeoutMs_ = timeout.count(); }

  void update() override
  {
    const int64_t timeoutMs = timeoutMs_.load();
    if (timeoutMs <= 0 || completed_)
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!watchdogStart_)
    {
      watchdogStart_ = now;
      return;
    }
    if (now - *watchdogStart_ > std::chrono::milliseconds(timeoutMs))
    {
      RCLCPP_ERROR(
        getLogger(), "[%s] Timed out after %ld ms without completing - posting failure",
        getName().c_str(), static_cast<long>(timeoutMs));
      this->postRgbFailure();
    }
  }

protected:
  virtual void wireCompletionSignals() {}

  void postRgbSuccess()
  {
    if (!completed_.exchange(true))
    {
      this->postSuccessEvent();
    }
  }

  void postRgbFailure()
  {
    if (!completed_.exchange(true))
    {
      this->postFailureEvent();
    }
  }

  bool completed() const { return completed_; }

  std::string cameraName_;
  CpCamera * camera_ = nullptr;
  CpTablePlane * table_ = nullptr;
  CpTableSegmenter * segmenter_ = nullptr;
  CpSceneSurvey * survey_ = nullptr;

private:
  std::atomic<bool> completed_{false};
  std::atomic<int64_t> timeoutMs_{0};  // 0 = watchdog disabled
  std::optional<std::chrono::steady_clock::time_point> watchdogStart_;
};

}  // namespace cl_rgb
