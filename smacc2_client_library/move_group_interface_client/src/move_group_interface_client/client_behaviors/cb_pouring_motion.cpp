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

#include <move_group_interface_client/client_behaviors/cb_circular_pivot_motion.hpp>
#include <move_group_interface_client/client_behaviors/cb_pouring_motion.hpp>
#include <move_group_interface_client/common.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_move_group_interface
{
CbCircularPouringMotion::CbCircularPouringMotion(
  const geometry_msgs::msg::Point & relativePivotPoint, double deltaHeight, std::string tipLink,
  std::string globalFrame)
: CbMoveEndEffectorTrajectory(tipLink),
  relativePivotPoint_(relativePivotPoint),
  deltaHeight_(deltaHeight),
  globalFrame_(globalFrame)

{
}

void CbCircularPouringMotion::generateTrajectory()
{
  // at least 1 sample per centimeter (average)
  const double METERS_PER_SAMPLE = 0.001;

  float dist_meters = 0;
  float totallineardist = fabs(this->deltaHeight_);

  int totalSamplesCount = totallineardist / METERS_PER_SAMPLE;
  int steps = totallineardist / METERS_PER_SAMPLE;

  float interpolation_factor_step = 1.0 / totalSamplesCount;

  double secondsPerSample;

  if (linearSpeed_m_s_)
  {
    secondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
  }
  else
  {
    secondsPerSample = std::numeric_limits<double>::max();
  }

  tf2::Stamped<tf2::Transform> currentEndEffectorTransform;

  this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);

  tf2::Transform lidEndEffectorTransform;
  tf2::fromMsg(this->pointerRelativePose_, lidEndEffectorTransform);

  tf2::Vector3 v0, v1;
  v0 = (currentEndEffectorTransform * lidEndEffectorTransform).getOrigin();

  tf2::Vector3 pivotPoint;
  tf2::fromMsg(this->relativePivotPoint_, pivotPoint);

  tf2::Vector3 pivot = (currentEndEffectorTransform * pivotPoint);

  v0 = v0 - pivot;
  v1 = v0;
  v1.setZ(v1.z() + this->deltaHeight_);

  tf2::Vector3 vp1(v1);
  tf2::Vector3 vp0(v0);
  tf2::Quaternion rotation = tf2::shortestArcQuatNormalize2(vp0, vp1);

  tf2::Quaternion initialEndEffectorOrientation = currentEndEffectorTransform.getRotation();
  auto finalEndEffectorOrientation = initialEndEffectorOrientation * rotation;

  tf2::Quaternion initialPointerOrientation =
    initialEndEffectorOrientation * lidEndEffectorTransform.getRotation();
  tf2::Quaternion finalPointerOrientation =
    finalEndEffectorOrientation * lidEndEffectorTransform.getRotation();

  // auto shortestAngle =
  //   tf2::angleShortestPath(initialEndEffectorOrientation, finalEndEffectorOrientation);

  v0 += pivot;
  v1 += pivot;

  float linc = deltaHeight_ / steps;  // METERS_PER_SAMPLE with sign
  float interpolation_factor = 0;
  tf2::Vector3 vi = v0;

  tf2::Transform invertedLidTransform = lidEndEffectorTransform.inverse();
  rclcpp::Time startTime = getNode()->now();

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] trajectory steps: " << steps);
  for (float i = 0; i < steps; i++)
  {
    // auto currentEndEffectorOrientation =
    //   tf2::slerp(initialEndEffectorOrientation, finalEndEffectorOrientation, interpolation_factor);
    auto currentPointerOrientation =
      tf2::slerp(initialPointerOrientation, finalPointerOrientation, interpolation_factor);

    interpolation_factor += interpolation_factor_step;
    dist_meters += linc;

    vi = v0;
    vi.setZ(vi.getZ() + dist_meters);

    tf2::Transform pose;
    pose.setOrigin(vi);
    pose.setRotation(currentPointerOrientation);

    geometry_msgs::msg::PoseStamped pointerPose;
    tf2::toMsg(pose, pointerPose.pose);
    pointerPose.header.frame_id = globalFrame_;
    pointerPose.header.stamp = startTime + rclcpp::Duration::from_seconds(i * secondsPerSample);
    this->pointerTrajectory_.push_back(pointerPose);

    tf2::Transform poseEndEffector = pose * invertedLidTransform;

    geometry_msgs::msg::PoseStamped globalEndEffectorPose;
    tf2::toMsg(poseEndEffector, globalEndEffectorPose.pose);
    globalEndEffectorPose.header.frame_id = globalFrame_;
    globalEndEffectorPose.header.stamp =
      startTime + rclcpp::Duration::from_seconds(i * secondsPerSample);

    this->endEffectorTrajectory_.push_back(globalEndEffectorPose);
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] " << i << " - " << globalEndEffectorPose);
  }
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "]trajectory generated, size: " << steps);
}

void CbCircularPouringMotion::createMarkers()
{
  CbMoveEndEffectorTrajectory::createMarkers();

  tf2::Stamped<tf2::Transform> currentEndEffectorTransform;
  this->getCurrentEndEffectorPose(globalFrame_, currentEndEffectorTransform);
  tf2::Vector3 pivotPoint;
  tf2::fromMsg(this->relativePivotPoint_, pivotPoint);
  tf2::Vector3 pivot = (currentEndEffectorTransform * pivotPoint);

  visualization_msgs::msg::Marker marker;

  marker.ns = "trajectory";
  marker.id = beahiorMarkers_.markers.size();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0;
  marker.color.b = 1.0;

  tf2::toMsg(pivot, marker.pose.position);
  marker.header.frame_id = globalFrame_;
  marker.header.stamp = getNode()->now();

  beahiorMarkers_.markers.push_back(marker);

  tf2::Transform localdirection;
  localdirection.setIdentity();
  localdirection.setOrigin(tf2::Vector3(0.05, 0, 0));
  auto frameid = this->pointerTrajectory_.front().header.frame_id;

  for (auto & pose : this->pointerTrajectory_)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frameid;
    marker.header.stamp = getNode()->now();
    marker.ns = "trajectory";
    marker.id = this->beahiorMarkers_.markers.size();
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.005;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1;
    marker.color.b = 0.0;

    geometry_msgs::msg::Point start, end;
    start.x = 0;
    start.y = 0;
    start.z = 0;

    tf2::Transform basetransform;
    tf2::fromMsg(pose.pose, basetransform);
    // tf2::Transform endarrow = localdirection * basetransform;

    end.x = localdirection.getOrigin().x();
    end.y = localdirection.getOrigin().y();
    end.z = localdirection.getOrigin().z();

    marker.pose.position = pose.pose.position;
    marker.pose.orientation = pose.pose.orientation;
    marker.points.push_back(start);
    marker.points.push_back(end);

    beahiorMarkers_.markers.push_back(marker);
  }
}

}  // namespace cl_move_group_interface
