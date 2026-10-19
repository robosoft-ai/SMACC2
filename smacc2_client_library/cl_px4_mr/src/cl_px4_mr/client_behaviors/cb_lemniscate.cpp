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

#include <cl_px4_mr/client_behaviors/cb_lemniscate.hpp>

namespace cl_px4_mr
{

CbLemniscate::CbLemniscate(FlightPatternFigureEightParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

CbLemniscate::CbLemniscate(int count, float size) : CbPx4PathFollowerBase(PathFollowerParams{})
{
  params_.count = count;
  params_.size = size;
}

std::vector<NedPoint> CbLemniscate::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = YawMode::TANGENT;
  RCLCPP_INFO(
    getLogger(), "CbLemniscate: %d x %s figure eight(s), lobe %.1f m, centre (%.1f, %.1f), ~%.0f m",
    params_.count, turnName(params_.direction), params_.size,
    std::isnan(params_.centerX) ? current.x : params_.centerX,
    std::isnan(params_.centerY) ? current.y : params_.centerY,
    flightPatternFigureEightLength(params_));
  return generateFlightPatternFigureEight(params_, current);
}

}  // namespace cl_px4_mr
