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

#include <tf2_ros/transform_listener.h>
#include <move_group_interface_client/client_behaviors/cb_circular_pivot_motion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace cl_move_group_interface
{
CbCircularPivotMotion::CbCircularPivotMotion(std::optional<std::string> tipLink)
: CbMoveEndEffectorTrajectory(tipLink)
{
}

CbCircularPivotMotion::CbCircularPivotMotion(
  const geometry_msgs::msg::PoseStamped & planePivotPose, double deltaRadians,
  std::optional<std::string> tipLink)
: CbMoveEndEffectorTrajectory(tipLink), planePivotPose_(planePivotPose), deltaRadians_(deltaRadians)
{
  if (tipLink_) planePivotPose_.header.frame_id = *tipLink;
}

CbCircularPivotMotion::CbCircularPivotMotion(
  const geometry_msgs::msg::PoseStamped & planePivotPose,
  const geometry_msgs::msg::Pose & relativeInitialPose, double deltaRadians,
  std::optional<std::string> tipLink)
: CbMoveEndEffectorTrajectory(tipLink),
  relativeInitialPose_(relativeInitialPose),
  planePivotPose_(planePivotPose),
  deltaRadians_(deltaRadians)
{
}

void CbCircularPivotMotion::generateTrajectory()
{
  if (!relativeInitialPose_)
  {
    this->computeCurrentEndEffectorPoseRelativeToPivot();
  }

  // project offset into the xy-plane
  // get the radius
  double radius = sqrt(
    relativeInitialPose_->position.z * relativeInitialPose_->position.z +
    relativeInitialPose_->position.y * relativeInitialPose_->position.y);
  double initialAngle = atan2(relativeInitialPose_->position.z, relativeInitialPose_->position.y);

  double totallineardist = fabs(radius * deltaRadians_);
  double totalangulardist = fabs(deltaRadians_);

  // at least 1 sample per centimeter (average)
  // at least 1 sample per ~1.1 degrees (average)

  const double RADS_PER_SAMPLE = 0.02;
  const double METERS_PER_SAMPLE = 0.01;

  int totalSamplesCount =
    std::max(totallineardist / METERS_PER_SAMPLE, totalangulardist / RADS_PER_SAMPLE);

  double linearSecondsPerSample;
  double angularSecondsPerSamples;
  double secondsPerSample;

  if (linearSpeed_m_s_)
  {
    linearSecondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
  }
  else
  {
    linearSecondsPerSample = std::numeric_limits<double>::max();
  }

  if (angularSpeed_rad_s_)
  {
    angularSecondsPerSamples = RADS_PER_SAMPLE / (*angularSpeed_rad_s_);
  }
  else
  {
    angularSecondsPerSamples = std::numeric_limits<double>::max();
  }

  if (!linearSpeed_m_s_ && !angularSpeed_rad_s_)
  {
    secondsPerSample = 0.5;
  }
  else
  {
    secondsPerSample = std::min(linearSecondsPerSample, angularSecondsPerSamples);
  }

  double currentAngle = initialAngle;

  double angleStep = deltaRadians_ / (double)totalSamplesCount;

  tf2::Transform tfBasePose;
  tf2::fromMsg(planePivotPose_.pose, tfBasePose);

  RCLCPP_INFO_STREAM(
    getLogger(),
    "[" << getName() << "] generated trajectory, total samples: " << totalSamplesCount);
  for (int i = 0; i < totalSamplesCount; i++)
  {
    // relativePose i
    double y = radius * cos(currentAngle);
    double z = radius * sin(currentAngle);

    geometry_msgs::msg::Pose relativeCurrentPose;

    relativeCurrentPose.position.x = relativeInitialPose_->position.x;
    relativeCurrentPose.position.y = y;
    relativeCurrentPose.position.z = z;

    tf2::Quaternion localquat;
    localquat.setEuler(currentAngle, 0, 0);

    //relativeCurrentPose.orientation = relativeInitialPose_.orientation;
    //tf2::toMsg(localquat, relativeCurrentPose.orientation);
    relativeCurrentPose.orientation.w = 1;

    tf2::Transform tfRelativeCurrentPose;
    tf2::fromMsg(relativeCurrentPose, tfRelativeCurrentPose);

    tf2::Transform tfGlobalPose = tfRelativeCurrentPose * tfBasePose;

    tfGlobalPose.setRotation(tfGlobalPose.getRotation() * localquat);

    geometry_msgs::msg::PoseStamped globalPose;
    tf2::toMsg(tfGlobalPose, globalPose.pose);
    globalPose.header.frame_id = planePivotPose_.header.frame_id;
    globalPose.header.stamp = rclcpp::Time(planePivotPose_.header.stamp) +
                              rclcpp::Duration::from_seconds(i * secondsPerSample);
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "]" << rclcpp::Time(globalPose.header.stamp).nanoseconds());

    this->endEffectorTrajectory_.push_back(globalPose);
    currentAngle += angleStep;
  }
}

void CbCircularPivotMotion::computeCurrentEndEffectorPoseRelativeToPivot()
{
  //auto currentRobotEndEffectorPose = this->movegroupClient_->moveGroupClientInterface.getCurrentPose();

  tf2_ros::Buffer tfBuffer(getNode()->get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);

  // tf2::Stamped<tf2::Transform>  globalBaseLink;
  tf2::Stamped<tf2::Transform> endEffectorInPivotFrame;

  try
  {
    if (!tipLink_ || *tipLink_ == "")
    {
      tipLink_ = this->movegroupClient_->moveGroupClientInterface->getEndEffectorLink();
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] waiting transform, pivot: '"
                       << planePivotPose_.header.frame_id << "' tipLink: '" << *tipLink_ << "'");
    tf2::fromMsg(
      tfBuffer.lookupTransform(
        planePivotPose_.header.frame_id, *tipLink_, rclcpp::Time(), rclcpp::Duration(10s)),
      endEffectorInPivotFrame);

    //endEffectorInPivotFrame = tfBuffer.lookupTransform(planePivotPose_.header.frame_id, *tipLink_, rclcpp::Time(0));

    // we define here the global frame as the pivot frame id
    // tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, rclcpp::Time(0), rclcpp::Duration(10));
    // tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, rclcpp::Time(0), globalBaseLink);
  }
  catch (const std::exception & e)
  {
    std::cerr << e.what() << '\n';
  }

  // tf2::Transform endEffectorInBaseLinkFrame;
  // tf2::fromMsg(currentRobotEndEffectorPose.pose, endEffectorInBaseLinkFrame);

  // tf2::Transform endEffectorInPivotFrame = globalBaseLink * endEffectorInBaseLinkFrame; // pose composition

  // now pivot and EndEffector share a common reference frame (let say map)
  // now get the current pose from the pivot reference frame with inverse composition
  tf2::Transform pivotTransform;
  tf2::fromMsg(planePivotPose_.pose, pivotTransform);
  tf2::Transform invertedNewReferenceFrame = pivotTransform.inverse();

  tf2::Transform currentPoseRelativeToPivot = invertedNewReferenceFrame * endEffectorInPivotFrame;

  geometry_msgs::msg::Pose finalEndEffectorRelativePose;
  tf2::toMsg(currentPoseRelativeToPivot, finalEndEffectorRelativePose);
  relativeInitialPose_ = finalEndEffectorRelativePose;
}

void CbCircularPivotMotion::createMarkers()
{
  CbMoveEndEffectorTrajectory::createMarkers();

  tf2::Transform localdirection;
  localdirection.setIdentity();
  localdirection.setOrigin(tf2::Vector3(0.12, 0, 0));
  auto frameid = planePivotPose_.header.frame_id;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frameid;
  marker.header.stamp = getNode()->now();
  marker.ns = "trajectory";
  marker.id = beahiorMarkers_.markers.size();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0;
  marker.color.b = 1.0;

  geometry_msgs::msg::Point start, end;
  start.x = planePivotPose_.pose.position.x;
  start.y = planePivotPose_.pose.position.y;
  start.z = planePivotPose_.pose.position.z;

  tf2::Transform basetransform;
  tf2::fromMsg(planePivotPose_.pose, basetransform);
  tf2::Transform endarrow = localdirection * basetransform;

  end.x = endarrow.getOrigin().x();
  end.y = endarrow.getOrigin().y();
  end.z = endarrow.getOrigin().z();

  marker.pose.orientation.w = 1;
  marker.points.push_back(start);
  marker.points.push_back(end);

  beahiorMarkers_.markers.push_back(marker);
}

}  // namespace cl_move_group_interface
