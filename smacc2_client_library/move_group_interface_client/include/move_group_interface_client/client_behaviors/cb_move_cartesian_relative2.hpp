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

#pragma once

#include <move_group_interface_client/cl_movegroup.hpp>
#include <tf2/transform_datatypes.h>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_move_group_interface
{
class CbMoveCartesianRelative2 : public CbMoveEndEffectorTrajectory
{
public:
  geometry_msgs::msg::Vector3 offset_;

  std::optional<double> linearSpeed_m_s_;

  CbMoveCartesianRelative2(std::string referenceFrame, std::string tipLink)
  : CbMoveEndEffectorTrajectory(tipLink)
  {
    globalFrame_ = referenceFrame;
  }

  CbMoveCartesianRelative2(
    std::string referenceFrame, std::string tipLink, geometry_msgs::msg::Vector3 offset)
  : CbMoveEndEffectorTrajectory(tipLink)
  {
    globalFrame_ = referenceFrame;
    offset_ = offset;
  }

  virtual void generateTrajectory() override
  {
    // at least 1 sample per centimeter (average)
    const double METERS_PER_SAMPLE = 0.001;

    float dist_meters = 0;
    tf2::Vector3 voffset;
    tf2::fromMsg(offset_, voffset);

    float totallineardist = voffset.length();

    int totalSamplesCount = totallineardist / METERS_PER_SAMPLE;
    int steps = totallineardist / METERS_PER_SAMPLE;

    float interpolation_factor_step = 1.0 / totalSamplesCount;

    double secondsPerSample;

    if (!linearSpeed_m_s_)
    {
      linearSpeed_m_s_ = 0.1;
    }

    secondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);

    tf2::Stamped<tf2::Transform> currentEndEffectorTransform;

    this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);

    tf2::Transform finalEndEffectorTransform = currentEndEffectorTransform;
    finalEndEffectorTransform.setOrigin(finalEndEffectorTransform.getOrigin() + voffset);

    float linc = totallineardist / steps;  // METERS_PER_SAMPLE with sign
    float interpolation_factor = 0;

    rclcpp::Time startTime = getNode()->now();

    for (float i = 0; i < steps; i++)
    {
      interpolation_factor += interpolation_factor_step;
      dist_meters += linc;

      tf2::Vector3 vi = currentEndEffectorTransform.getOrigin().lerp(
        finalEndEffectorTransform.getOrigin(), interpolation_factor);

      tf2::Transform pose;
      pose.setOrigin(vi);
      pose.setRotation(currentEndEffectorTransform.getRotation());

      geometry_msgs::msg::PoseStamped pointerPose;
      pointerPose.pose = tf2::toMsg(pose);
      pointerPose.header.frame_id = globalFrame_;
      pointerPose.header.stamp = startTime + rclcpp::Duration(i * secondsPerSample);

      this->endEffectorTrajectory_.push_back(pointerPose);
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[CbMoveEndEffectorRelative2] Target End efector Pose: "
                     << this->endEffectorTrajectory_.back());
  }

private:
  std::string globalFrame_;
};
}  // namespace cl_move_group_interface
