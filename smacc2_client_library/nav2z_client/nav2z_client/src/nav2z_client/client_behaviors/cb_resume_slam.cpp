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
#include <nav2z_client/client_behaviors/cb_resume_slam.hpp>

namespace cl_nav2z
{
CbResumeSlam::CbResumeSlam(std::string serviceName)
: smacc2::client_behaviors::CbServiceCall<slam_toolbox::srv::Pause>(serviceName.c_str())
{
}

void CbResumeSlam::onEntry()
{
  this->requiresComponent(this->slam_);

  auto currentState = slam_->getState();

  if (currentState == CpSlamToolbox::SlamToolboxState::Paused)
  {
    RCLCPP_INFO(
      getLogger(), "[CbResumeSlam] calling pause service to toggle from paused to resumed");
    this->request_ = std::make_shared<slam_toolbox::srv::Pause::Request>();
    smacc2::client_behaviors::CbServiceCall<slam_toolbox::srv::Pause>::onEntry();
    this->slam_->toogleState();
  }
  else
  {
    this->request_ = nullptr;
    RCLCPP_INFO(
      getLogger(), "[CbResumeSlam] calling skipped. The current slam state is already resumed.");
  }
}

}  // namespace cl_nav2z
