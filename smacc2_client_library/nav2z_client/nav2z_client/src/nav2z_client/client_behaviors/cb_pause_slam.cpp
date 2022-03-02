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
#include <nav2z_client/client_behaviors/cb_pause_slam.hpp>

namespace cl_nav2z
{
CbPauseSlam::CbPauseSlam(std::string serviceName)
: smacc2::client_behaviors::CbServiceCall<slam_toolbox::srv::Pause>(serviceName.c_str())
{
}

void CbPauseSlam::onEntry()
{
  this->requiresComponent(this->slam_);

  auto currentState = slam_->getState();

  if (currentState == CpSlamToolbox::SlamToolboxState::Resumed)
  {
    RCLCPP_INFO(
      getLogger(), "[CbPauseSlam] calling pause service to toggle from resumed to paused");
    this->request_ = std::make_shared<slam_toolbox::srv::Pause::Request>();
    CbServiceCall<slam_toolbox::srv::Pause>::onEntry();
    this->slam_->toogleState();
  }
  else
  {
    this->request_ = nullptr;
    RCLCPP_INFO(
      getLogger(), "[CbPauseSlam] calling skipped. The current slam state is already paused.");
  }
}

}  // namespace cl_nav2z
