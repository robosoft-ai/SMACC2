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

#include <cl_nav2z/components/costmap_switch/cp_costmap_switch.hpp>

namespace cl_nav2z
{
std::array<std::string, 4> CpCostmapSwitch::layerNames = {
  "global_costmap/obstacles_layer",
  "local_costmap/obstacles_layer"
  "global_costmap/inflater_layer",
  "local_costmap/inflater_layer"};

void CpCostmapSwitch::registerProxyFromDynamicReconfigureServer(
  std::string costmapName, std::string enablePropertyName)
{
  RCLCPP_INFO(getLogger(), "[CpCostmapSwitch] registering costmap type: %s", costmapName.c_str());
  auto proxy = std::make_shared<CpCostmapProxy>(
    this->nav2zClient_->getName() + "/" + costmapName, enablePropertyName, getNode());
  costmapProxies[costmapName] = proxy;
}

CpCostmapSwitch::CpCostmapSwitch() {}

void CpCostmapSwitch::onInitialize()
{
  this->nav2zClient_ = dynamic_cast<cl_nav2z::ClNav2Z *>(owner_);

  if (this->nav2zClient_ == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "the owner of the CpCostmapSwitch must be a ClNav2Z");
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

std::string CpCostmapSwitch::getStandardCostmapName(StandardLayers layertype)
{
  return CpCostmapSwitch::layerNames[(int)layertype];
}

bool CpCostmapSwitch::exists(std::string layerName)
{
  if (!this->costmapProxies.count(layerName))
  {
    return false;
  }

  return true;
}

void CpCostmapSwitch::enable(std::string layerName)
{
  RCLCPP_INFO(getLogger(), "[CpCostmapSwitch] enabling %s", layerName.c_str());

  if (!exists(layerName))
  {
    RCLCPP_ERROR(getLogger(), "[CpCostmapSwitch] costmap %s does not exist", layerName.c_str());
    return;
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[CpCostmapSwitch] costmap %s found. Calling dynamic reconfigure server.",
      layerName.c_str());
    costmapProxies[layerName]->setCostmapEnabled(true);
  }
}

void CpCostmapSwitch::enable(StandardLayers layerType)
{
  this->enable(getStandardCostmapName(layerType));
}

void CpCostmapSwitch::disable(std::string layerName)
{
  RCLCPP_INFO(getLogger(), "[CpCostmapSwitch] disabling %s", layerName.c_str());

  if (!exists(layerName))
  {
    RCLCPP_ERROR(getLogger(), "[CpCostmapSwitch] costmap %s does not exist", layerName.c_str());
    return;
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[CpCostmapSwitch] costmap %s found. Calling dynamic reconfigure server.",
      layerName.c_str());
    costmapProxies[layerName]->setCostmapEnabled(false);
  }
}

void CpCostmapSwitch::disable(StandardLayers layerType)
{
  this->disable(getStandardCostmapName(layerType));
}

//-------------------------------------------------------------------------

CpCostmapProxy::CpCostmapProxy(
  std::string /*costmap_name*/, std::string /*enablePropertyName*/, rclcpp::Node::SharedPtr nh)
: nh_(nh)
{
  RCLCPP_ERROR(nh->get_logger(), "costmap switch not implemented %s", costmapName_.c_str());
}

void CpCostmapProxy::setCostmapEnabled(bool /*value*/)
{
  RCLCPP_ERROR(nh_->get_logger(), "costmap switch not implemented %s", costmapName_.c_str());
}
}  // namespace cl_nav2z
