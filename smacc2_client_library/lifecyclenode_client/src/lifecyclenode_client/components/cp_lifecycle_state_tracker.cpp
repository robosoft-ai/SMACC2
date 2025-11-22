// Copyright 2024 RobosoftAI Inc.
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

#include <lifecyclenode_client/components/cp_lifecycle_state_tracker.hpp>

namespace cl_lifecyclenode
{
void CpLifecycleStateTracker::onInitialize()
{
  // Initialize with empty state
  RCLCPP_INFO(getLogger(), "[CpLifecycleStateTracker] Component initialized");
}

void CpLifecycleStateTracker::updateState(const lifecycle_msgs::msg::State & state)
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  currentState_ = state;
  RCLCPP_DEBUG(
    getLogger(), "[CpLifecycleStateTracker] State updated to: %s (id: %d)", state.label.c_str(),
    state.id);
}

std::optional<lifecycle_msgs::msg::State> CpLifecycleStateTracker::getCurrentState() const
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  return currentState_;
}

std::optional<std::string> CpLifecycleStateTracker::getCurrentStateLabel() const
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (currentState_)
  {
    return currentState_->label;
  }
  return std::nullopt;
}

uint8_t CpLifecycleStateTracker::getCurrentStateId() const
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (currentState_)
  {
    return currentState_->id;
  }
  return 0;
}

}  // namespace cl_lifecyclenode
