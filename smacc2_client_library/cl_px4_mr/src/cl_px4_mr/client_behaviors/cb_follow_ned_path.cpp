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

#include <cl_px4_mr/client_behaviors/cb_follow_ned_path.hpp>

#include <algorithm>

namespace cl_px4_mr
{

CbFollowNedPath::CbFollowNedPath(std::vector<NedPoint> path, PathFollowerParams follower)
: CbPx4PathFollowerBase(follower), path_(std::move(path))
{
}

void CbFollowNedPath::setPath(std::vector<NedPoint> path)
{
  path_ = std::move(path);
  // the base drops degenerate segments, which would shift reachedCount()
  for (size_t i = 1; i < path_.size(); ++i)
  {
    if (nedDistance(path_[i - 1], path_[i]) < followerParams_.minSegmentLength)
    {
      RCLCPP_WARN(
        getLogger(),
        "CbFollowNedPath: vertices %zu and %zu are closer than minSegmentLength (%.2f m) - the "
        "follower will merge them and reachedCount() will be off by one from there on",
        i - 1, i, followerParams_.minSegmentLength);
    }
  }
}

size_t CbFollowNedPath::reachedCount() const
{
  if (path_.empty())
  {
    return 0;
  }
  const size_t idx = carrotVertexIndex();
  // with the entry position prepended, followed vertex i is route vertex i-1
  const size_t reached = followerParams_.prependCurrentPosition ? idx : idx + 1;
  return std::min(reached, path_.size());
}

}  // namespace cl_px4_mr
