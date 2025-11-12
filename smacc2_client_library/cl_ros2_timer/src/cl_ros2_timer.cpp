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

#include <cl_ros2_timer/cl_ros2_timer.hpp>

namespace cl_ros2_timer
{
ClRos2Timer::ClRos2Timer(rclcpp::Duration duration, bool oneshot)
: duration_(duration), oneshot_(oneshot)
{
}

ClRos2Timer::~ClRos2Timer()
{
  // Components are automatically cleaned up by the framework
}

}  // namespace cl_ros2_timer
