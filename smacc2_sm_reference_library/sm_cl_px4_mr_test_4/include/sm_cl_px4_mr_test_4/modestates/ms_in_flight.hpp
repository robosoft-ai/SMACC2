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

#include <cl_px4_mr/cl_px4_mr.hpp>
#include <cl_px4_mr/components/cp_kml_mission_loader.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_plan.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>

namespace sm_cl_px4_mr_test_4
{

// MODE STATE: airborne, executing the railway mission.
//
// Owns the MissionPlan: built once here (container onEntry runs before the
// first inner state is constructed) and read by StRailway, the nav states and
// the pattern superstates through context<MsInFlight>().plan. It survives
// every inner transition because MsInFlight itself never exits until landing.
struct MsInFlight : smacc2::SmaccState<MsInFlight, SmClPx4MrTest4, StAscend>
{
  using SmaccState::SmaccState;

  railway::MissionPlan plan;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsInFlight ---");

    std::string testLeg;
    this->getGlobalSMData("test_leg", testLeg);

    if (!testLeg.empty())
    {
      plan = railway::buildTestLegPlan(testLeg, railway::kMissionAltitudeM, getLogger());
    }
    else
    {
      cl_px4_mr::ClPx4Mr * px4Client = nullptr;
      this->requiresClient(px4Client);
      cl_px4_mr::CpKmlMissionLoader * loader =
        px4Client ? px4Client->getComponent<cl_px4_mr::CpKmlMissionLoader>() : nullptr;
      cl_px4_mr::CpVehicleLocalPosition * localPosition =
        px4Client ? px4Client->getComponent<cl_px4_mr::CpVehicleLocalPosition>() : nullptr;

      const bool haveRef = localPosition != nullptr && localPosition->globalRefValid();
      railway::Projector project = [localPosition, haveRef](double lat, double lon, float & x, float & y) {
        return haveRef && localPosition->projectToNed(lat, lon, x, y);
      };
      railway::Reprojector reproject = [localPosition, haveRef](float x, float y, double & lat, double & lon) {
        return haveRef && localPosition->reprojectFromNed(x, y, lat, lon);
      };
      const std::vector<cl_px4_mr::GeoPoint> backbone =
        loader != nullptr && loader->hasMission() ? loader->getMission()
                                                  : std::vector<cl_px4_mr::GeoPoint>{};
      const std::string source = loader != nullptr ? loader->getSource() : "none";

      if (railway::kLayout == railway::Layout::DEMO_RING)
      {
        plan = railway::buildDemoRingPlan(
          backbone, project, reproject, railway::kMissionAltitudeM, railway::kLandingSite, source,
          getLogger());
      }
      else if (backbone.empty())
      {
        RCLCPP_ERROR(getLogger(), "MsInFlight: no mission backbone - plan invalid");
        plan.valid = false;
      }
      else if (!haveRef)
      {
        RCLCPP_ERROR(
          getLogger(), "MsInFlight: PX4 global reference (ref_lat/ref_lon) not valid - plan invalid");
        plan.valid = false;
      }
      else
      {
        plan = railway::buildMissionPlan(
          backbone, project, railway::kMissionAltitudeM, source, getLogger());
      }
    }

    RCLCPP_INFO(getLogger(), "%s", plan.toTable().c_str());
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsInFlight ---");
  }
};

}  // namespace sm_cl_px4_mr_test_4
