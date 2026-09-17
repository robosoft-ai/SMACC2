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

#include <smacc2/smacc.hpp>

#include <cl_px4_mr/client_behaviors/cb_load_kml_mission.hpp>
#include <cl_px4_mr/components/cp_kml_mission_loader.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: read the backbone LineString from config/Hormuz_3.kml (or the
// compiled-in fallback) into CpKmlMissionLoader
struct StLoadMission : smacc2::SmaccState<StLoadMission, MsDisarmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLoadKmlMission, OrPx4>, StArmPX4, SUCCESS>,
    Transition<EvCbFailure<CbLoadKmlMission, OrPx4>, StMissionAborted, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLoadKmlMission>(
      railway::kmlSharePath(), railway::fallbackBackbone());
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StLoadMission: loading %s", railway::kmlSharePath().c_str());
  }

  void onExit()
  {
    ClPx4Mr * px4Client = nullptr;
    this->requiresClient(px4Client);
    CpKmlMissionLoader * loader = px4Client ? px4Client->getComponent<CpKmlMissionLoader>() : nullptr;
    if (loader != nullptr && loader->hasMission())
    {
      RCLCPP_INFO(
        getLogger(), "StLoadMission: backbone has %zu vertices (source: %s)",
        loader->getMission().size(), loader->getSource().c_str());
    }
  }
};

}  // namespace sm_cl_px4_mr_test_4
