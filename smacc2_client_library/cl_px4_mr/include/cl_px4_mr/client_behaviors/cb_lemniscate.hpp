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

#pragma once

#include <cl_px4_mr/client_behaviors/cb_px4_path_follower_base.hpp>
#include <cl_px4_mr/utils/pattern_generators.hpp>

namespace cl_px4_mr
{

// Figure eight (lemniscate of Bernoulli) about a point (default: the current
// position) on the path follower: entered at the nearest point of the curve,
// flown `count` times at the follower's ground speed, yaw along the path so a
// forward camera films where the vehicle is going. Unlike CbFigureEight (which
// streams setpoints with a fixed heading) it is leashed and terminates on
// arrival at the last vertex.
class CbLemniscate : public CbPx4PathFollowerBase
{
public:
  explicit CbLemniscate(
    FlightPatternFigureEightParams params = {}, PathFollowerParams follower = {});
  explicit CbLemniscate(int count, float size = 5.0f);

  void setParams(const FlightPatternFigureEightParams & params) { params_ = params; }
  const FlightPatternFigureEightParams & params() const { return params_; }
  void setCenter(float x, float y)
  {
    params_.centerX = x;
    params_.centerY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbLemniscate"; }

private:
  FlightPatternFigureEightParams params_;
};

}  // namespace cl_px4_mr
