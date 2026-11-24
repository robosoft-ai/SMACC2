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

#include <cl_rgb/client_behaviors/cb_rgb_client_behavior_base.hpp>

#include <atomic>
#include <cstdint>
#include <string>

namespace cl_rgb
{

struct WaitForFrameParams
{
  std::string camera = "cam_head";
  unsigned int frames = 1;  // succeed on this many images newer than the state's entry
};

// Succeeds when the camera has delivered new images since the state was entered: the way
// to make sure the next state looks at the scene as it is now, not as it was.
class CbWaitForFrame : public CbRgbClientBehaviorBase
{
public:
  explicit CbWaitForFrame(const WaitForFrameParams & params = WaitForFrameParams());

  void onEntry() override;
  void onExit() override {}

protected:
  void wireCompletionSignals() override;

private:
  void onFrame(const FramePtr & frame);

  WaitForFrameParams params_;
  std::atomic<uint64_t> startSequence_{0};
};

}  // namespace cl_rgb
