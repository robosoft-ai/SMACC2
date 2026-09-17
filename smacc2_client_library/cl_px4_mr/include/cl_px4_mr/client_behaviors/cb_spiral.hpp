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

// Archimedean spiral about a centre (default: current position) between two
// radii, outward or inward, at constant altitude. Unlike CbSpiralPattern it
// follows the path at a commanded ground speed (leashed) and terminates on
// arrival at the last vertex.
class CbSpiral : public CbPx4PathFollowerBase
{
public:
  explicit CbSpiral(FlightPatternSpiralParams params = {}, PathFollowerParams follower = {});

  void setParams(const FlightPatternSpiralParams & params) { params_ = params; }
  const FlightPatternSpiralParams & params() const { return params_; }
  void setCenter(float x, float y)
  {
    params_.centerX = x;
    params_.centerY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbSpiral"; }

private:
  FlightPatternSpiralParams params_;
};

}  // namespace cl_px4_mr
