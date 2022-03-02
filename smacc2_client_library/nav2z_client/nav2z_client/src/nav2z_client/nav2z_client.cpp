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
#include <nav2z_client/nav2z_client.hpp>

#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace cl_nav2z
{
typedef smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose> Base;
typedef Base::WrappedResult WrappedResult;

ClNav2Z::ClNav2Z(std::string moveBaseName) : Base(moveBaseName)
{
  // RCLCPP_INFO(getLogger(),"Smacc Move Base Action Client");
}

ClNav2Z::~ClNav2Z() {}
}  // namespace cl_nav2z

PLUGINLIB_EXPORT_CLASS(cl_nav2z::ClNav2Z, smacc2::ISmaccClient)
