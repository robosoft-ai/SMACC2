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

#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client.hpp>

#include <cl_mission_tracker/components/cp_decision_manager.hpp>

namespace cl_mission_tracker
{

// Event: Battery load decision
template <typename AsyncCB, typename Orthogonal>
struct EvBatteryLoad : sc::event<EvBatteryLoad<AsyncCB, Orthogonal>>
{
};

// Event: Radial motion decision
template <typename AsyncCB, typename Orthogonal>
struct EvRadialMotion : sc::event<EvRadialMotion<AsyncCB, Orthogonal>>
{
};

// Event: S-pattern decision
template <typename AsyncCB, typename Orthogonal>
struct EvSPattern : sc::event<EvSPattern<AsyncCB, Orthogonal>>
{
};

// Event: F-pattern decision
template <typename AsyncCB, typename Orthogonal>
struct EvFPattern : sc::event<EvFPattern<AsyncCB, Orthogonal>>
{
};

/**
 * @brief Client for mission tracking and decision sequencing.
 *
 * This client follows the pure component-based architecture pattern.
 * It acts as an orchestrator that creates and wires components:
 *
 * - CpDecisionManager: Manages decision counter state
 *
 * Client behaviors should use requiresComponent() to access the
 * decision manager, not direct client fields.
 */
class ClMissionTracker : public smacc2::ISmaccClient
{
public:
  ClMissionTracker() {}

  virtual ~ClMissionTracker() {}

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Create decision manager component
    this->createComponent<CpDecisionManager, TOrthogonal, ClMissionTracker>();
  }
};

}  // namespace cl_mission_tracker
