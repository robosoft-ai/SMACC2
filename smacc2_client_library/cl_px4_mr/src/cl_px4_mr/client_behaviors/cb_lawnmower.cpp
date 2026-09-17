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

/*****************************************************************************************************************
 *
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#include <cl_px4_mr/client_behaviors/cb_lawnmower.hpp>

namespace cl_px4_mr
{

CbLawnmower::CbLawnmower(FlightPatternLawnmowerParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

CbLawnmower::CbLawnmower(float laneHeading) : CbPx4PathFollowerBase(PathFollowerParams{})
{
  params_.laneHeading = laneHeading;
}

std::vector<NedPoint> CbLawnmower::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = YawMode::TANGENT;
  RCLCPP_INFO(
    getLogger(),
    "CbLawnmower: %d lanes x %.0f m, spacing %.0f m, heading %.2f rad, %s at (%.1f, %.1f), %.0f m",
    flightPatternLawnmowerLaneCount(params_), params_.laneLength, params_.laneSpacing,
    std::isnan(params_.laneHeading) ? current.yaw : params_.laneHeading,
    params_.originIsCenter ? "centred" : "cornered",
    std::isnan(params_.originX) ? current.x : params_.originX,
    std::isnan(params_.originY) ? current.y : params_.originY, flightPatternLawnmowerLength(params_));
  return generateFlightPatternLawnmower(params_, current);
}

}  // namespace cl_px4_mr
