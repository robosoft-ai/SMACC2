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
 *****************************************************************************************************************/

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cl_moveit2z/common.hpp>
#include <cl_moveit2z/components/cp_tf_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cb_circular_pivot_motion.hpp"

using namespace std::chrono_literals;

namespace cl_moveit2z
{
// spins the end effector joint (or any other arbitrary joint etting the tipLink parameter)
class CbEndEffectorRotate : public CbCircularPivotMotion
{
public:
  CbEndEffectorRotate(double deltaRadians, std::optional<std::string> tipLink = std::nullopt)
  : CbCircularPivotMotion(tipLink)
  {
    deltaRadians_ = deltaRadians;
  }

  virtual ~CbEndEffectorRotate() {}

  virtual void onEntry() override
  {
    // Use CpTfListener component for transform lookups
    CpTfListener * tfListener = nullptr;
    this->requiresComponent(tfListener, false);  // Optional component

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
        auto pivotFrameName =
          this->movegroupClient_->moveGroupClientInterface->getEndEffectorLink();

        if (tfListener != nullptr)
        {
          // Use component-based TF listener (preferred)
          auto transformOpt =
            tfListener->lookupTransform(pivotFrameName, *tipLink_, rclcpp::Time());

          if (transformOpt)
          {
            tf2::fromMsg(transformOpt.value(), endEffectorInPivotFrame);
            tf2::toMsg(endEffectorInPivotFrame, this->planePivotPose_.pose);
            this->planePivotPose_.header.frame_id = endEffectorInPivotFrame.frame_id_;
            this->planePivotPose_.header.stamp =
              rclcpp::Time(endEffectorInPivotFrame.stamp_.time_since_epoch().count());
            break;
          }
          else
          {
            RCLCPP_ERROR_STREAM(
              getLogger(), "[" << getName() << "] Failed to lookup transform from " << *tipLink_
                               << " to " << pivotFrameName << ". Attempt countdown: " << attempts);
            rclcpp::sleep_for(500ms);
            attempts--;
          }
        }
        else
        {
          // Fallback to legacy TF2 usage if component not available
          RCLCPP_WARN_STREAM(
            getLogger(),
            "[" << getName()
                << "] CpTfListener component not available, using legacy TF2 (consider "
                   "adding CpTfListener component)");
          tf2_ros::Buffer tfBuffer(getNode()->get_clock());
          tf2_ros::TransformListener tfListenerLegacy(tfBuffer);

          tf2::fromMsg(
            tfBuffer.lookupTransform(
              pivotFrameName, *tipLink_, rclcpp::Time(), rclcpp::Duration(10s)),
            endEffectorInPivotFrame);

          tf2::toMsg(endEffectorInPivotFrame, this->planePivotPose_.pose);
          this->planePivotPose_.header.frame_id = endEffectorInPivotFrame.frame_id_;
          this->planePivotPose_.header.stamp =
            rclcpp::Time(endEffectorInPivotFrame.stamp_.time_since_epoch().count());
          break;
        }
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR_STREAM(getLogger(), e.what() << ". Attempt countdown: " << attempts);
        rclcpp::sleep_for(500ms);
        attempts--;
      }
    }

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] pivotPose: " << planePivotPose_);

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] calling base CbCircularPivotMotion::onEntry");
    CbCircularPivotMotion::onEntry();
  }

  std::optional<std::string> tipLink;
};

}  // namespace cl_moveit2z
