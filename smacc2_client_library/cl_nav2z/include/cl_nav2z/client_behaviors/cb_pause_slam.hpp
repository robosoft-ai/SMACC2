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

#include <cl_nav2z/components/slam_toolbox/cp_slam_toolbox.hpp>
#include <slam_toolbox/srv/pause.hpp>
#include <smacc2/client_behaviors/cb_call_service.hpp>

namespace cl_nav2z
{
class CbPauseSlam : public smacc2::client_behaviors::CbServiceCall<slam_toolbox::srv::Pause>
{
public:
  CbPauseSlam(std::string serviceName = "/slam_toolbox/pause_new_measurements")
  : smacc2::client_behaviors::CbServiceCall<slam_toolbox::srv::Pause>(serviceName.c_str())
  {
  }

  inline void onEntry() override
  {
    this->requiresComponent(this->slam_);

    auto currentState = slam_->getState();

    if (currentState == CpSlamToolbox::SlamToolboxState::Resumed)
    {
      RCLCPP_INFO(
        getLogger(), "[CbPauseSlam] calling pause service to toggle from resumed to paused");
      this->request_ = std::make_shared<slam_toolbox::srv::Pause::Request>();
      CbServiceCall<slam_toolbox::srv::Pause>::onEntry();
      this->slam_->toggleState();
    }
    else
    {
      this->request_ = nullptr;
      RCLCPP_INFO(
        getLogger(), "[CbPauseSlam] calling skipped. The current slam state is already paused.");
    }
  }

protected:
  CpSlamToolbox * slam_;
};
}  // namespace cl_nav2z
