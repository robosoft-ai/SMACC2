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

#include <cl_px4_mr/client_behaviors/cb_sine_wave_horizontal.hpp>

namespace cl_px4_mr
{

CbSineWaveHorizontal::CbSineWaveHorizontal(
  FlightPatternSineWaveHorizontalParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

std::vector<NedPoint> CbSineWaveHorizontal::buildPath(const NedPoint & current)
{
  NedPoint end;
  end.x = std::isnan(params_.endX) ? current.x : params_.endX;
  end.y = std::isnan(params_.endY) ? current.y : params_.endY;

  followerParams_.yawMode = YawMode::FIXED;
  followerParams_.fixedYaw = legHeading(current, end);
  followerParams_.prependCurrentPosition = false;  // path starts at the entry point

  RCLCPP_INFO(
    getLogger(), "CbSineWaveHorizontal: to (%.1f, %.1f), A=%.1f m, lambda=%.1f m, heading %.2f rad",
    end.x, end.y, params_.amplitude, params_.wavelength, followerParams_.fixedYaw);
  return generateFlightPatternSineWaveHorizontal(params_, current);
}

}  // namespace cl_px4_mr
