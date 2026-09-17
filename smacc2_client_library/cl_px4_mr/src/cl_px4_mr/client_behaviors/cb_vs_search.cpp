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

#include <cl_px4_mr/client_behaviors/cb_vs_search.hpp>

namespace cl_px4_mr
{

CbVSSearch::CbVSSearch(FlightPatternVSSearchParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

std::vector<NedPoint> CbVSSearch::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = YawMode::TANGENT;
  RCLCPP_INFO(
    getLogger(), "CbVSSearch: %s sector search, r=%.0f m, %d cycle(s), datum (%.1f, %.1f), %.0f m",
    turnName(params_.direction), params_.radius, params_.cycles,
    std::isnan(params_.datumX) ? current.x : params_.datumX,
    std::isnan(params_.datumY) ? current.y : params_.datumY, flightPatternVSSearchLength(params_));
  return generateFlightPatternVSSearch(params_, current);
}

}  // namespace cl_px4_mr
