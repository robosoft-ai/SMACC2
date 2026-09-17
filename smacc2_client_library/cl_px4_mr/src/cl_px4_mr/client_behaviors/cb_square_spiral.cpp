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

#include <cl_px4_mr/client_behaviors/cb_square_spiral.hpp>

namespace cl_px4_mr
{

CbSquareSpiral::CbSquareSpiral(FlightPatternSquareSpiralParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

CbSquareSpiral::CbSquareSpiral(Turn direction) : CbPx4PathFollowerBase(PathFollowerParams{})
{
  params_.direction = direction;
}

std::vector<NedPoint> CbSquareSpiral::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = YawMode::TANGENT;
  RCLCPP_INFO(
    getLogger(), "CbSquareSpiral: %s, %d legs, spacing %.1f m, origin (%.1f, %.1f), %.0f m total",
    turnName(params_.direction), params_.numLegs, params_.spacing,
    std::isnan(params_.originX) ? current.x : params_.originX,
    std::isnan(params_.originY) ? current.y : params_.originY,
    flightPatternSquareSpiralLength(params_));
  return generateFlightPatternSquareSpiral(params_, current);
}

}  // namespace cl_px4_mr
