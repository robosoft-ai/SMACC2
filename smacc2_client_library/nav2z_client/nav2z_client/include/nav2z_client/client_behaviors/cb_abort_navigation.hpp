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

#include <tf2_ros/buffer.h>

#include "cb_nav2z_client_behavior_base.hpp"
#include "cb_navigate_global_position.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace cl_nav2z
{
class CbAbortNavigation : public CbNav2ZClientBehaviorBase
{
public:
  CbAbortNavigation();

  void onEntry() override;
  void onExit() override;

  void onNavigationActionSuccess(const ClNav2Z::WrappedResult &) override;
  void onNavigationActionAbort(const ClNav2Z::WrappedResult &) override;
};
}  // namespace cl_nav2z
