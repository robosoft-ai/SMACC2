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
#include <nav2z_client/client_behaviors/cb_nav2z_client_behavior_base.hpp>

namespace cl_nav2z
{
CbNav2ZClientBehaviorBase::~CbNav2ZClientBehaviorBase() {}

void CbNav2ZClientBehaviorBase::propagateSuccessEvent(ClNav2Z::WrappedResult & r)
{
  navigationResult_ = r.code;
  auto name = smacc2::demangleType(typeid(*this));
  RCLCPP_INFO(getLogger(), "[%s] Propagating success event from action server", name.c_str());
  this->postSuccessEvent();
}
void CbNav2ZClientBehaviorBase::propagateFailureEvent(ClNav2Z::WrappedResult & r)
{
  navigationResult_ = r.code;
  auto name = smacc2::demangleType(typeid(*this));
  RCLCPP_INFO(getLogger(), "[%s] Propagating failure event from action server", name.c_str());
  this->postFailureEvent();
}
}  // namespace cl_nav2z
