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

#pragma once

#include <cl_px4_mr/client_behaviors/cb_px4_path_follower_base.hpp>
#include <cl_px4_mr/utils/pattern_generators.hpp>

namespace cl_px4_mr
{

// Rate-controlled climb (or descent) to an altitude at the current XY (or a
// target XY, for a precise pre-landing descent), heading held. Succeeds immediately if already within tolerance. Unlike
// CbChangeAltitude (single setpoint step + goal checker) the setpoint climbs
// at `climbRate` and stays leashed to the vehicle.
class CbAscendToAltitude : public CbPx4PathFollowerBase
{
public:
  explicit CbAscendToAltitude(FlightPatternAscendParams params = {}, PathFollowerParams follower = {});
  explicit CbAscendToAltitude(float altitudeAgl, float climbRate = 1.5f);

  void setParams(const FlightPatternAscendParams & params) { params_ = params; }
  void setTargetXy(float x, float y)
  {
    params_.targetX = x;
    params_.targetY = y;
  }
  const FlightPatternAscendParams & params() const { return params_; }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbAscendToAltitude"; }

private:
  FlightPatternAscendParams params_;
};

}  // namespace cl_px4_mr
