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

#pragma once

#include <smacc2/component.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace cl_lifecycle_node
{
/**
 * @brief Component that tracks the current lifecycle state
 *
 * CpLifecycleStateTracker maintains knowledge of the current lifecycle node
 * state, providing thread-safe query methods for state information.
 */
class CpLifecycleStateTracker : public smacc2::ISmaccComponent
{
public:
  CpLifecycleStateTracker() = default;
  virtual ~CpLifecycleStateTracker() = default;

  /**
   * @brief Update the current state
   * @param state New lifecycle state
   */
  void updateState(const lifecycle_msgs::msg::State & state);

  /**
   * @brief Get the current lifecycle state
   * @return Optional state (empty if not yet initialized)
   */
  std::optional<lifecycle_msgs::msg::State> getCurrentState() const;

  /**
   * @brief Get the current state label as string
   * @return Optional state label (empty if not yet initialized)
   */
  std::optional<std::string> getCurrentStateLabel() const;

  /**
   * @brief Get the current state ID
   * @return State ID (0 if not yet initialized)
   */
  uint8_t getCurrentStateId() const;

  /**
   * @brief Component initialization
   */
  void onInitialize() override;

private:
  std::optional<lifecycle_msgs::msg::State> currentState_;
  mutable std::mutex stateMutex_;
};

}  // namespace cl_lifecycle_node
