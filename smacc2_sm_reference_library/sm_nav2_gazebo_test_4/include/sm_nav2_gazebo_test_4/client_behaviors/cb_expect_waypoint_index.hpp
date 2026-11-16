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

#include <cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace sm_nav2_gazebo_test_4
{
// A check of this test machine: succeeds when the waypoint navigator's index is the expected one.
class CbExpectWaypointIndex : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbExpectWaypointIndex(long expected) : expected_(expected) {}

  void onEntry() override
  {
    cl_nav2z::CpWaypointNavigator * navigator = nullptr;
    this->requiresComponent(navigator, smacc2::ComponentRequirement::HARD);
    const long index = navigator->getCurrentWaypointIndex();
    RCLCPP_INFO(getLogger(), "[CbExpectWaypointIndex] index %ld, expected %ld", index, expected_);
    if (index == expected_)
    {
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[CbExpectWaypointIndex] the waypoint index is not the expected one");
      this->postFailureEvent();
    }
  }

private:
  long expected_;
};
}  // namespace sm_nav2_gazebo_test_4
