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

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <move_base_z_client_plugin/move_base_z_client_plugin.hpp>
#include <smacc2/component.hpp>

// #include <dynamic_reconfigure/DoubleParameter.h>
// #include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/Config.h>

namespace cl_move_base_z
{
class CostmapProxy;

class CostmapSwitch : public smacc2::ISmaccComponent
{
public:
  enum class StandardLayers
  {
    GLOBAL_OBSTACLES_LAYER = 0,
    LOCAL_OBSTACLES_LAYER = 1,
    GLOBAL_INFLATED_LAYER = 2,
    LOCAL_INFLATED_LAYER = 3
  };

  static std::array<std::string, 4> layerNames;

  CostmapSwitch();

  void onInitialize() override;

  static std::string getStandardCostmapName(StandardLayers layertype);

  bool exists(std::string layerName);

  void enable(std::string layerName);

  void enable(StandardLayers layerType);

  void disable(std::string layerName);

  void disable(StandardLayers layerType);

  void registerProxyFromDynamicReconfigureServer(
    std::string costmapName, std::string enablePropertyName = "enabled");

private:
  std::map<std::string, std::shared_ptr<CostmapProxy>> costmapProxies;
  cl_move_base_z::ClMoveBaseZ * moveBaseClient_;
};
//-------------------------------------------------------------------------
class CostmapProxy
{
public:
  CostmapProxy(
    std::string costmap_name, std::string enablePropertyName, rclcpp::Node::SharedPtr nh);

  void setCostmapEnabled(bool value);

private:
  std::string costmapName_;
  rclcpp::Node::SharedPtr nh_;

  inline rclcpp::Node::SharedPtr getNode() { return nh_; }
  // dynamic_reconfigure::Config enableReq;
  // dynamic_reconfigure::Config disableReq;

  // void dynreconfCallback(const dynamic_reconfigure::Config::ConstPtr &configuration_update);

  // ros::Subscriber dynrecofSub_;
};
}  // namespace cl_move_base_z
