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
 *****************************************************************************************************************/

#pragma once

#include <tf2/transform_datatypes.h>
#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/common.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cb_move_end_effector_trajectory.hpp"

using namespace std::chrono_literals;

namespace cl_moveit2z
{
class CbMoveCartesianRelative2 : public CbMoveEndEffectorTrajectory
{
public:
  geometry_msgs::msg::Vector3 offset_;

  std::optional<double> linearSpeed_m_s_;

  CbMoveCartesianRelative2(std::string referenceFrame, std::string tipLink)
  : CbMoveEndEffectorTrajectory(std::move(tipLink)), globalFrame_(std::move(referenceFrame))
  {
  }

  CbMoveCartesianRelative2(
    std::string referenceFrame, std::string tipLink, geometry_msgs::msg::Vector3 offset)
  : CbMoveEndEffectorTrajectory(std::move(tipLink)),
    offset_(offset),
    globalFrame_(std::move(referenceFrame))
  {
  }

  virtual ~CbMoveCartesianRelative2() {}

  void generateTrajectory() override
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] generating trajectory");
    // at least 1 sample per centimeter (average)
    const double METERS_PER_SAMPLE = 0.001;

    tf2::Vector3 voffset;
    tf2::fromMsg(offset_, voffset);

    float totallineardist = static_cast<float>(voffset.length());

    int totalSamplesCount = static_cast<int>(totallineardist / METERS_PER_SAMPLE);
    int steps = totalSamplesCount;

    float interpolation_factor_step = static_cast<float>(1.0 / totalSamplesCount);

    double secondsPerSample;

    if (!linearSpeed_m_s_)
    {
      linearSpeed_m_s_ = 0.1;
    }

    secondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);

    tf2::Stamped<tf2::Transform> currentEndEffectorTransform;
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] getting current end effector pose");
    this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);

    tf2::Transform finalEndEffectorTransform = currentEndEffectorTransform;
    finalEndEffectorTransform.setOrigin(finalEndEffectorTransform.getOrigin() + voffset);

    float interpolation_factor = 0;

    rclcpp::Time startTime = getNode()->now();
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] trajectory steps: " << steps);

    for (int i = 0; i < steps; i++)
    {
      interpolation_factor += interpolation_factor_step;

      tf2::Vector3 vi = currentEndEffectorTransform.getOrigin().lerp(
        finalEndEffectorTransform.getOrigin(), interpolation_factor);

      tf2::Transform pose;
      pose.setOrigin(vi);
      pose.setRotation(currentEndEffectorTransform.getRotation());

      geometry_msgs::msg::PoseStamped pointerPose;
      tf2::toMsg(pose, pointerPose.pose);
      pointerPose.header.frame_id = globalFrame_;
      pointerPose.header.stamp = startTime + rclcpp::Duration::from_seconds(i * secondsPerSample);

      this->endEffectorTrajectory_.push_back(pointerPose);
    }

    RCLCPP_INFO_STREAM(
      getLogger(),
      "[" << getName() << "] Target End efector Pose: " << this->endEffectorTrajectory_.back());
  }

private:
  std::string globalFrame_;
};
}  // namespace cl_moveit2z
