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

#include <cl_px4_mr/client_behaviors/cb_loiter.hpp>

namespace cl_px4_mr
{

CbLoiter::CbLoiter(FlightPatternLoiterParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

CbLoiter::CbLoiter(int count, float radius) : CbPx4PathFollowerBase(PathFollowerParams{})
{
  params_.count = count;
  params_.radius = radius;
}

std::vector<NedPoint> CbLoiter::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = params_.faceCenter ? YawMode::PER_VERTEX : YawMode::TANGENT;
  RCLCPP_INFO(
    getLogger(), "CbLoiter: %d x %s circle(s), r=%.1f m, centre (%.1f, %.1f)", params_.count,
    turnName(params_.direction), params_.radius,
    std::isnan(params_.centerX) ? current.x : params_.centerX,
    std::isnan(params_.centerY) ? current.y : params_.centerY);
  return generateFlightPatternLoiter(params_, current);
}

}  // namespace cl_px4_mr
