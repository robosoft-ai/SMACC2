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
#include <smacc2/client_behaviors/cb_ros_launch_2.hpp>

namespace smacc2
{
namespace client_behaviors
{
std::vector<std::future<std::string>> CbRosLaunch2::detached_futures_;

CbRosLaunch2::CbRosLaunch2()
: packageName_(std::nullopt),
  launchFileName_(std::nullopt),
  launchMode_(RosLaunchMode::LAUNCH_CLIENT_BEHAVIOR_LIFETIME)
{
}

CbRosLaunch2::CbRosLaunch2(std::string package, std::string launchfile, RosLaunchMode launchmode)
: packageName_(package), launchFileName_(launchfile), launchMode_(launchmode)
{
}

CbRosLaunch2::~CbRosLaunch2() {}

template <typename TOrthogonal, typename TSourceObject>
void onOrthogonalAllocation()
{
  smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
}

void CbRosLaunch2::onEntry()
{
  RCLCPP_INFO_STREAM(getLogger(), "[CbRosLaunch2] OnEntry");

  std::string packageName, launchFileName;
  if (launchFileName_ && packageName_)
  {
    std::function<bool()> breakfunction;

    if (launchMode_ == RosLaunchMode::LAUNCH_CLIENT_BEHAVIOR_LIFETIME)
    {
      // breakfunction = [this]() -> bool { 
      //   auto ret = this->isShutdownRequested(); 
      //   return ret;
      //   };

      breakfunction = std::bind(&CbRosLaunch2::isShutdownRequested, this);
    }
    else
    {
      breakfunction = []() -> bool { return false; };
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[CbRosLaunch2] launching: " << *packageName_ << " , " << *launchFileName_
                                               << "LaunchMode: " << (int)launchMode_);

    auto fut = smacc2::client_bases::ClRosLaunch2::executeRosLaunch(*packageName_, *launchFileName_, breakfunction);

    // if (launchMode_ == RosLaunchMode::LAUNCH_DETTACHED)
    //   detached_futures_.push_back(std::move(fut));
    // else
    //   future_ = std::move(fut);
  }
  else
  {
    RCLCPP_INFO_STREAM(getLogger(), "[CbRosLaunch2] finding Ros Launch client");

    this->requiresClient(client_);
    if (client_ != nullptr)
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[CbRosLaunch2] launching from client: " << client_->packageName_ << " , "
                                                             << client_->launchFileName_);

      client_->launch();
    }
    else
    {
      RCLCPP_ERROR(
        getLogger(),
        "[CbRosLaunch2] Inccorrect ros launch operation. No Ros Launch client specified neither "
        "package/roslaunch file path.");
    }
  }
}

}  // namespace client_behaviors
}  // namespace smacc2
