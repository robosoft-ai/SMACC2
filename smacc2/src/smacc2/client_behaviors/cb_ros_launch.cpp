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
#include <smacc2/client_behaviors/cb_ros_launch.hpp>

namespace smacc2
{
namespace client_behaviors
{
CbRosLaunch::CbRosLaunch() {}

// CbRosLaunch::CbRosLaunch(std::string packageName, std::string launchFileName)
// : packageName_(packageName), launchFileName_(launchFileName)
// {
// }

CbRosLaunch::~CbRosLaunch() {}

template <typename TOrthogonal, typename TSourceObject>
void onOrthogonalAllocation()
{
  smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
}

void CbRosLaunch::onEntry()
{
  RCLCPP_INFO_STREAM(getLogger(), "[CbRosLaunch] OnEntry");

  std::string packageName, launchFileName;
  if (launchFileName_ && packageName_)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[CbRosLaunch] launching: " << *packageName_ << " , " << *launchFileName_);
    smacc2::client_bases::ClRosLaunch::executeRosLaunch(
      *packageName_, *launchFileName_, []() { return false; });
  }
  else
  {
    RCLCPP_INFO_STREAM(getLogger(), "[CbRosLaunch] finding Ros Launch client");

    this->requiresClient(client_);
    if (client_ != nullptr)
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[CbRosLaunch] launching from client: " << client_->packageName_ << " , "
                                                             << client_->launchFileName_);

      client_->launch();
    }
    else
    {
      RCLCPP_ERROR(
        getLogger(),
        "[CbRosLaunch] Inccorrect ros launch operation. No Ros Launch client specified neither "
        "package/roslaunch file path.");
    }
  }
}

}  // namespace client_behaviors
}  // namespace smacc2
