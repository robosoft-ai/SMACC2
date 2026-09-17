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

// Boustrophedon lawnmower over a rectangle centred on (or cornered at) an
// origin, lanes along `laneHeading`. Posts success at the end of the last lane.
class CbLawnmower : public CbPx4PathFollowerBase
{
public:
  explicit CbLawnmower(FlightPatternLawnmowerParams params = {}, PathFollowerParams follower = {});
  explicit CbLawnmower(float laneHeading);

  void setParams(const FlightPatternLawnmowerParams & params) { params_ = params; }
  const FlightPatternLawnmowerParams & params() const { return params_; }
  void setOrigin(float x, float y)
  {
    params_.originX = x;
    params_.originY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbLawnmower"; }

private:
  FlightPatternLawnmowerParams params_;
};

}  // namespace cl_px4_mr
