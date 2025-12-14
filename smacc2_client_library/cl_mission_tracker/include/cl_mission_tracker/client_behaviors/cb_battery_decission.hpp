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

#include <cl_mission_tracker/cl_mission_tracker.hpp>
#include <cl_mission_tracker/components/cp_decision_manager.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_mission_tracker
{

/**
 * @brief Behavior that makes mission decisions based on a counter.
 *
 * This behavior uses requiresComponent() to access:
 * - CpDecisionManager: For decision counter state
 *
 * Based on the current decision count, it posts different events
 * to control the mission flow.
 */
class CbBatteryDecission : public smacc2::SmaccAsyncClientBehavior
{
private:
  // Component accessed via requiresComponent (NOT direct client!)
  CpDecisionManager * decisionManager_ = nullptr;
  std::function<void()> postEventFn_;

public:
  CbBatteryDecission() {}

  virtual ~CbBatteryDecission() {}

  virtual void onEntry() override { this->postEventFn_(); }

  virtual void onExit() override {}

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postEventFn_ = [this]()
    {
      // Get component via requiresComponent pattern (NOT requiresClient!)
      this->requiresComponent(decisionManager_, true);

      // Get current decision and advance counter
      int decisionCount = decisionManager_->nextDecision();

      switch (decisionCount)
      {
        case 0:
        case 2:
        case 4:
          this->postEvent<EvBatteryLoad<TSourceObject, TOrthogonal>>();
          break;
        case 1:
          this->postEvent<EvRadialMotion<TSourceObject, TOrthogonal>>();
          break;
        case 3:
          this->postEvent<EvSPattern<TSourceObject, TOrthogonal>>();
          break;
        case 5:
          this->postEvent<EvFPattern<TSourceObject, TOrthogonal>>();
          break;
        default:
          break;
      }
    };
  }
};

}  // namespace cl_mission_tracker
