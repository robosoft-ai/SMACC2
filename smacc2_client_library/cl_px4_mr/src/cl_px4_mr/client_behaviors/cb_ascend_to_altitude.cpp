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

#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>

#include <algorithm>

namespace cl_px4_mr
{

CbAscendToAltitude::CbAscendToAltitude(
  FlightPatternAscendParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

CbAscendToAltitude::CbAscendToAltitude(float altitudeAgl, float climbRate)
: CbPx4PathFollowerBase(PathFollowerParams{})
{
  params_.altitudeAgl = altitudeAgl;
  params_.climbRate = climbRate;
}

std::vector<NedPoint> CbAscendToAltitude::buildPath(const NedPoint & current)
{
  // vertical move: speed = climb rate, keep heading, no prepend (path already
  // starts at the current position), leash sized for the climb rate
  followerParams_.groundSpeed = std::max(params_.climbRate, 0.1f);
  followerParams_.leash = std::max(followerParams_.leash, 2.0f * followerParams_.groundSpeed);
  followerParams_.yawMode = YawMode::HOLD_ENTRY;
  followerParams_.prependCurrentPosition = false;
  followerParams_.arrivalZTol = std::min(followerParams_.arrivalZTol, params_.tolerance);

  RCLCPP_INFO(
    getLogger(), "CbAscendToAltitude: %.1f m AGL -> %.1f m AGL at %.1f m/s", -current.z,
    params_.altitudeAgl, followerParams_.groundSpeed);
  return generateFlightPatternAscend(params_, current);
}

}  // namespace cl_px4_mr
