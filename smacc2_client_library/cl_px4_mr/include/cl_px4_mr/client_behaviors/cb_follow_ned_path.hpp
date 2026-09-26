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

#include <vector>

namespace cl_px4_mr
{

// Streams a caller-supplied NED polyline through the carrot follower: one
// continuous, leashed flight along a route of arbitrary vertices (a cave
// passage, a survey line, a retrace of an earlier route) instead of a chain
// of CbGoToLocation states or CbFollowWaypoints' setpoint jumps.
//
// The route is given at construction or via setPath() from the owning state's
// runtimeConfigure (state machine thread, before onEntry). reachedCount() is
// the number of route vertices the carrot has passed; a state reads it in its
// onExit (state machine thread, updates barred) to record how far the vehicle
// got, e.g. to retrace only the traversed prefix on an abort.
class CbFollowNedPath : public CbPx4PathFollowerBase
{
public:
  explicit CbFollowNedPath(std::vector<NedPoint> path = {}, PathFollowerParams follower = {});

  void setPath(std::vector<NedPoint> path);
  const std::vector<NedPoint> & path() const { return path_; }

  // route vertices passed by the carrot, 0..path().size(); == path().size()
  // once the path completed
  size_t reachedCount() const;

protected:
  std::vector<NedPoint> buildPath(const NedPoint & /*current*/) override { return path_; }
  const char * behaviorName() const override { return "CbFollowNedPath"; }

private:
  std::vector<NedPoint> path_;
};

}  // namespace cl_px4_mr
