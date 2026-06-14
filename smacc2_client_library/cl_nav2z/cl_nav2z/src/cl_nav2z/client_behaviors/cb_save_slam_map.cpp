// Copyright 2025 Robosoft Inc.
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

#include <angles/angles.h>
#include <chrono>
#include <cl_nav2z/client_behaviors/cb_save_slam_map.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav2_msgs/srv/save_map.hpp>
#include <slam_toolbox/srv/save_map.hpp>
#include <smacc2/client_behaviors/cb_call_service.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <std_msgs/msg/string.hpp>

namespace cl_nav2z
{
using namespace std::chrono_literals;

CbSaveSlamMap::CbSaveSlamMap() : CbServiceCall("/map_saver/save_map", getRequest()) {}

void CbSaveSlamMap::onExit() {}

std::shared_ptr<nav2_msgs::srv::SaveMap::Request> CbSaveSlamMap::getRequest()
{
  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  request->map_topic = "map";
  request->map_url = "/tmp/saved_map";
  request->image_format = "png";
  request->occupied_thresh = 0.65;
  request->free_thresh = 0.25;
  request->map_mode = "trinary";

  return request;
}
}  // namespace cl_nav2z
