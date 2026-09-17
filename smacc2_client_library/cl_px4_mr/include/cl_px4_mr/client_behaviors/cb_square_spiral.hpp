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

// Expanding square spiral from an origin (default: current position): legs
// s, s, 2s, 2s, 3s, 3s ... turning RIGHT or LEFT at every corner. Posts
// success at the end of the last leg.
class CbSquareSpiral : public CbPx4PathFollowerBase
{
public:
  explicit CbSquareSpiral(
    FlightPatternSquareSpiralParams params = {}, PathFollowerParams follower = {});
  explicit CbSquareSpiral(Turn direction);

  void setParams(const FlightPatternSquareSpiralParams & params) { params_ = params; }
  const FlightPatternSquareSpiralParams & params() const { return params_; }
  void setOrigin(float x, float y)
  {
    params_.originX = x;
    params_.originY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbSquareSpiral"; }

private:
  FlightPatternSquareSpiralParams params_;
};

}  // namespace cl_px4_mr
