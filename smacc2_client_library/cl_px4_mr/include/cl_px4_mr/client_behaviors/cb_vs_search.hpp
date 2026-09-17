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

// Victor Sierra sector search about a datum (default: current position).
// Posts success back at the datum after the last leg.
class CbVSSearch : public CbPx4PathFollowerBase
{
public:
  explicit CbVSSearch(FlightPatternVSSearchParams params = {}, PathFollowerParams follower = {});

  void setParams(const FlightPatternVSSearchParams & params) { params_ = params; }
  const FlightPatternVSSearchParams & params() const { return params_; }
  void setDatum(float x, float y)
  {
    params_.datumX = x;
    params_.datumY = y;
  }

protected:
  std::vector<NedPoint> buildPath(const NedPoint & current) override;
  const char * behaviorName() const override { return "CbVSSearch"; }

private:
  FlightPatternVSSearchParams params_;
};

}  // namespace cl_px4_mr
