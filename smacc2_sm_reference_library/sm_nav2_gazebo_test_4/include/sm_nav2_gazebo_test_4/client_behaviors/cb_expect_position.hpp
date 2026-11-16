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

#include <cmath>

#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace sm_nav2_gazebo_test_4
{
// A check of this test machine: succeeds when the robot is within `tolerance` metres of (x, y) in the
// map frame, fails otherwise.
class CbExpectPosition : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbExpectPosition(double x, double y, double tolerance) : x_(x), y_(y), tolerance_(tolerance) {}

  void onEntry() override
  {
    cl_nav2z::CpPose * pose = nullptr;
    this->requiresComponent(pose, smacc2::ComponentRequirement::HARD);
    const auto current = pose->toPoseMsg();
    const double distance = std::hypot(current.position.x - x_, current.position.y - y_);
    RCLCPP_INFO(
      getLogger(), "[CbExpectPosition] at (%.2f, %.2f), expected (%.2f, %.2f), distance %.2f m",
      current.position.x, current.position.y, x_, y_, distance);
    if (distance <= tolerance_)
    {
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[CbExpectPosition] more than %.2f m away", tolerance_);
      this->postFailureEvent();
    }
  }

private:
  double x_;
  double y_;
  double tolerance_;
};
}  // namespace sm_nav2_gazebo_test_4
