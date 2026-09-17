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

#include <cl_px4_mr/client_behaviors/cb_sine_wave_vertical.hpp>

namespace cl_px4_mr
{

CbSineWaveVertical::CbSineWaveVertical(FlightPatternSineWaveVerticalParams params, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), params_(params)
{
}

std::vector<NedPoint> CbSineWaveVertical::buildPath(const NedPoint & current)
{
  followerParams_.yawMode = YawMode::TANGENT;
  followerParams_.prependCurrentPosition = false;  // path starts at the entry point

  const float peakVerticalSpeed = params_.amplitude * 2.0f * static_cast<float>(M_PI) *
                                  followerParams_.groundSpeed / std::max(params_.wavelength, 0.1f);
  if (peakVerticalSpeed > 2.0f)
  {
    RCLCPP_WARN(
      getLogger(),
      "CbSineWaveVertical: peak vertical speed %.2f m/s exceeds typical PX4 limits - the flown "
      "sine will flatten",
      peakVerticalSpeed);
  }
  RCLCPP_INFO(
    getLogger(), "CbSineWaveVertical: to (%.1f, %.1f), A=%.1f m, lambda=%.1f m", params_.endX,
    params_.endY, params_.amplitude, params_.wavelength);
  return generateFlightPatternSineWaveVertical(params_, current);
}

}  // namespace cl_px4_mr
