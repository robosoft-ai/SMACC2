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

// Crosshatch grid: a lawnmower pass then a second pass rotated 90 degrees over
// the same rectangle, centred on the origin. Posts success at the end of the
// second pass.
class CbGridPattern : public CbPx4PathFollowerBase
{
public:
  explicit CbGridPattern(
    FlightPatternGridPatternParams params = {}, PathFollowerParams follower = {});

  void setParams(const FlightPatternGridPatternParams & params) { params_ = params; }
  const FlightPatternGridPatternParams & params() const { return params_; }
  void setOrigin(float x, float y)
  {
    params_.base.originX = x;
    params_.base.originY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbGridPattern"; }

private:
  FlightPatternGridPatternParams params_;
};

}  // namespace cl_px4_mr
