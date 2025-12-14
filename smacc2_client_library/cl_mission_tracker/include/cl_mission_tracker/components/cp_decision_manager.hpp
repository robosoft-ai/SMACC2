// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc2/component.hpp>

#include <mutex>

namespace cl_mission_tracker
{

/**
 * @brief Component that manages mission decision state.
 *
 * This component tracks the decision counter for mission sequencing.
 * It provides thread-safe access to decision state, separating
 * mission logic from the client orchestrator.
 */
class CpDecisionManager : public smacc2::ISmaccComponent
{
public:
  CpDecisionManager() = default;

  virtual ~CpDecisionManager() = default;

  void onInitialize() override { RCLCPP_INFO(getLogger(), "[CpDecisionManager] Initialized"); }

  /**
   * @brief Get the current decision counter value.
   * @return Current decision count
   */
  int getDecisionCounter() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return decisionCounter_;
  }

  /**
   * @brief Increment the decision counter and return the previous value.
   * @return Decision count before incrementing
   */
  int nextDecision()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    int current = decisionCounter_;
    decisionCounter_++;
    RCLCPP_INFO_STREAM(
      getLogger(), "[CpDecisionManager] Decision " << current << " -> " << decisionCounter_);
    return current;
  }

  /**
   * @brief Reset the decision counter to zero.
   */
  void resetDecisionCounter()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    decisionCounter_ = 0;
    RCLCPP_INFO(getLogger(), "[CpDecisionManager] Decision counter reset");
  }

  /**
   * @brief Set the decision counter to a specific value.
   * @param value New counter value
   */
  void setDecisionCounter(int value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    decisionCounter_ = value;
    RCLCPP_INFO_STREAM(getLogger(), "[CpDecisionManager] Decision counter set to " << value);
  }

private:
  mutable std::mutex mutex_;
  int decisionCounter_ = 0;
};

}  // namespace cl_mission_tracker
