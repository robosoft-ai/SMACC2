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

#include <nav2z_client/components/slam_toolbox/cp_slam_toolbox.hpp>
#include <slam_toolbox/srv/pause.hpp>
#include <smacc2/client_behaviors/cb_call_service.hpp>

namespace cl_nav2z
{
class CbPauseSlam : public smacc2::client_behaviors::CbServiceCall<slam_toolbox::srv::Pause>
{
public:
  CbPauseSlam(std::string serviceName = "/slam_toolbox/pause_new_measurements");
  void onEntry() override;

protected:
  CpSlamToolbox * slam_;
};
}  // namespace cl_nav2z
