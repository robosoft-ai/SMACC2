// Copyright 2021 RobosoftAI Inc.
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
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <optional>
#include "cb_nav2z_client_behavior_base.hpp"

namespace cl_nav2z
{
using odom_tracker::OdomTracker;

template <typename TCbRelativeMotion>
class CbRetry : public TCbRelativeMotion
{
public:
  CbRetry() {}
  void onEntry() override
  {
    odomTracker_ = this->moveBaseClient_->template getComponent<OdomTracker>();
    auto goal = odomTracker_->getCurrentMotionGoal();

    if (goal)
    {
      this->goalPose_ = *goal;
    }

    TCbRelativeMotion::onEntry();
  }

private:
  OdomTracker * odomTracker_;
};
}  // namespace cl_nav2z
