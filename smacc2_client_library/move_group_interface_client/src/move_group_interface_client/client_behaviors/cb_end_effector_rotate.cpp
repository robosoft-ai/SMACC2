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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <move_group_interface_client/client_behaviors/cb_end_effector_rotate.hpp>
#include <move_group_interface_client/common.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace cl_move_group_interface
{
CbEndEffectorRotate::CbEndEffectorRotate(double deltaRadians, std::optional<std::string> tipLink)
: CbCircularPivotMotion(tipLink)
{
  deltaRadians_ = deltaRadians;
}

CbEndEffectorRotate::~CbEndEffectorRotate() {}

void CbEndEffectorRotate::onEntry()
{
  // autocompute pivot pose
  tf2_ros::Buffer tfBuffer(getNode()->get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);

  tf2::Stamped<tf2::Transform> endEffectorInPivotFrame;

  int attempts = 3;

  this->requiresClient(movegroupClient_);
  if (!tipLink_)
  {
    tipLink_ = this->movegroupClient_->moveGroupClientInterface->getEndEffectorLink();
    RCLCPP_WARN_STREAM(
      getLogger(),
      "[" << getName() << "] tip unspecified, using default end effector: " << *tipLink_);
  }

  while (attempts > 0)
  {
    try
    {
      //auto pivotFrameName = this->movegroupClient_->moveGroupClientInterface->getPlanningFrame();
      auto pivotFrameName = this->movegroupClient_->moveGroupClientInterface->getEndEffectorLink();

      tf2::Stamped<tf2::Transform> endEffectorInPivotFrame;

      tf2::fromMsg(
        tfBuffer.lookupTransform(pivotFrameName, *tipLink_, rclcpp::Time(), rclcpp::Duration(10s)),
        endEffectorInPivotFrame);

      tf2::toMsg(endEffectorInPivotFrame, this->planePivotPose_.pose);
      this->planePivotPose_.header.frame_id = endEffectorInPivotFrame.frame_id_;
      this->planePivotPose_.header.stamp =
        rclcpp::Time(endEffectorInPivotFrame.stamp_.time_since_epoch().count());
      break;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(getLogger(), e.what() << ". Attempt countdown: " << attempts);
      rclcpp::Duration(500ms);
      attempts--;
    }
  }

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] pivotPose: " << planePivotPose_);

  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] calling base CbCircularPivotMotion::onEntry");
  CbCircularPivotMotion::onEntry();
}
}  // namespace cl_move_group_interface
