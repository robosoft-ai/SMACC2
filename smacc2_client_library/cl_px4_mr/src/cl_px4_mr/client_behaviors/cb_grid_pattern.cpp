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

#include <cl_px4_mr/client_behaviors/cb_grid_pattern.hpp>

namespace cl_px4_mr
{

CbGridPattern::CbGridPattern(FlightPatternGridPatternParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

std::vector<NedPoint> CbGridPattern::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = YawMode::TANGENT;
  RCLCPP_INFO(
    getLogger(), "CbGridPattern: %.0f x %.0f m rectangle, spacing %.0f m, %s, centre (%.1f, %.1f), %.0f m",
    params_.base.laneLength, params_.base.width, params_.base.laneSpacing,
    params_.secondPass ? "two passes" : "single pass",
    std::isnan(params_.base.originX) ? current.x : params_.base.originX,
    std::isnan(params_.base.originY) ? current.y : params_.base.originY,
    flightPatternGridPatternLength(params_));
  return generateFlightPatternGridPattern(params_, current);
}

}  // namespace cl_px4_mr
