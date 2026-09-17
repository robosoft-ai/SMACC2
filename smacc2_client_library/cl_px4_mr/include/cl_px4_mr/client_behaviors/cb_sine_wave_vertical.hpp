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

// Transit from the current position to an end point with sinusoidal altitude
// about the base altitude; heading along the leg. Posts success on arrival at
// the end point (at base altitude).
class CbSineWaveVertical : public CbPx4PathFollowerBase
{
public:
  explicit CbSineWaveVertical(
    FlightPatternSineWaveVerticalParams params = {}, PathFollowerParams follower = {});

  void setParams(const FlightPatternSineWaveVerticalParams & params) { params_ = params; }
  const FlightPatternSineWaveVerticalParams & params() const { return params_; }
  void setEndPoint(float x, float y)
  {
    params_.endX = x;
    params_.endY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbSineWaveVertical"; }

private:
  FlightPatternSineWaveVerticalParams params_;
};

}  // namespace cl_px4_mr
