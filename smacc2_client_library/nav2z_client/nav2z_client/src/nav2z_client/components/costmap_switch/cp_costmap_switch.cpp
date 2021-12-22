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

#include <nav2z_client/components/costmap_switch/cp_costmap_switch.hpp>

namespace cl_nav2z
{
std::array<std::string, 4> CostmapSwitch::layerNames = {
  "global_costmap/obstacles_layer",
  "local_costmap/obstacles_layer"
  "global_costmap/inflater_layer",
  "local_costmap/inflater_layer"};

void CostmapSwitch::registerProxyFromDynamicReconfigureServer(
  std::string costmapName, std::string enablePropertyName)
{
  RCLCPP_INFO(getLogger(), "[CostmapSwitch] registering costmap type: %s", costmapName.c_str());
  auto proxy = std::make_shared<CostmapProxy>(
    this->moveBaseClient_->getName() + "/" + costmapName, enablePropertyName, getNode());
  costmapProxies[costmapName] = proxy;
}

CostmapSwitch::CostmapSwitch() {}

void CostmapSwitch::onInitialize()
{
  this->moveBaseClient_ = dynamic_cast<cl_nav2z::ClNav2Z *>(owner_);

  if (this->moveBaseClient_ == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "the owner of the CostmapSwitch must be a ClNav2Z");
  }

  registerProxyFromDynamicReconfigureServer(
    getStandardCostmapName(StandardLayers::GLOBAL_OBSTACLES_LAYER));
  registerProxyFromDynamicReconfigureServer(
    getStandardCostmapName(StandardLayers::LOCAL_OBSTACLES_LAYER));
  registerProxyFromDynamicReconfigureServer(
    getStandardCostmapName(StandardLayers::GLOBAL_INFLATED_LAYER));
  registerProxyFromDynamicReconfigureServer(
    getStandardCostmapName(StandardLayers::LOCAL_INFLATED_LAYER));
}

std::string CostmapSwitch::getStandardCostmapName(StandardLayers layertype)
{
  return CostmapSwitch::layerNames[(int)layertype];
}

bool CostmapSwitch::exists(std::string layerName)
{
  if (!this->costmapProxies.count(layerName))
  {
    return false;
  }

  return true;
}

void CostmapSwitch::enable(std::string layerName)
{
  RCLCPP_INFO(getLogger(), "[CostmapSwitch] enabling %s", layerName.c_str());

  if (!exists(layerName))
  {
    RCLCPP_ERROR(getLogger(), "[CostmapSwitch] costmap %s does not exist", layerName.c_str());
    return;
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[CostmapSwitch] costmap %s found. Calling dynamic reconfigure server.",
      layerName.c_str());
    costmapProxies[layerName]->setCostmapEnabled(true);
  }
}

void CostmapSwitch::enable(StandardLayers layerType)
{
  this->enable(getStandardCostmapName(layerType));
}

void CostmapSwitch::disable(std::string layerName)
{
  RCLCPP_INFO(getLogger(), "[CostmapSwitch] disabling %s", layerName.c_str());

  if (!exists(layerName))
  {
    RCLCPP_ERROR(getLogger(), "[CostmapSwitch] costmap %s does not exist", layerName.c_str());
    return;
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[CostmapSwitch] costmap %s found. Calling dynamic reconfigure server.",
      layerName.c_str());
    costmapProxies[layerName]->setCostmapEnabled(false);
  }
}

void CostmapSwitch::disable(StandardLayers layerType)
{
  this->disable(getStandardCostmapName(layerType));
}

//-------------------------------------------------------------------------

CostmapProxy::CostmapProxy(
  std::string /*costmap_name*/, std::string /*enablePropertyName*/, rclcpp::Node::SharedPtr nh)
: nh_(nh)
{
  // this->costmapName_ = costmap_name + "/set_parameters";
  // dynamic_reconfigure::BoolParameter enableField;
  // enableField.name = "enabled";
  // enableField.value = true;

  // enableReq.bools.push_back(enableField);

  // enableField.value = false;
  // disableReq.bools.push_back(enableField);
  RCLCPP_ERROR(nh->get_logger(), "costmap switch not implemented %s", costmapName_.c_str());
}

void CostmapProxy::setCostmapEnabled(bool /*value*/)
{
  // dynamic_reconfigure::ReconfigureRequest srv_req;
  // dynamic_reconfigure::ReconfigureResponse srv_resp;

  // if (value)
  //     srv_req.config = enableReq;
  // else
  //     srv_req.config = disableReq;

  // if (ros::service::exists(costmapName_, true))
  // {
  //     RCLCPP_INFO(getLogger(),"sending dynamic reconfigure request: %s", costmapName_.c_str());
  //     ros::service::call(costmapName_, srv_req, srv_resp);
  // }
  // else
  // {
  //     RCLCPP_WARN(getLogger(),"could not call dynamic reconfigure server. It does not exist: %s", costmapName_.c_str());
  // }

  RCLCPP_ERROR(nh_->get_logger(), "costmap switch not implemented %s", costmapName_.c_str());
}

// void CostmapProxy::dynreconfCallback(const dynamic_reconfigure::Config::ConstPtr &configuration_update)
// {
//     // auto gp = std::find_if(configuration_update->strs.begin(), configuration_update->strs.begin(),
//     //                        [&](const dynamic_reconfigure::StrParameter &p) { return p.name == "base_global_planner" && p.value == desired_global_planner_; });
// }
}  // namespace cl_nav2z
