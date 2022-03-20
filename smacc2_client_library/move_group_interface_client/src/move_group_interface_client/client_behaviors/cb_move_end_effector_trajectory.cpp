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

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.hpp>
#include <move_group_interface_client/common.hpp>
#include <move_group_interface_client/components/cp_trajectory_history.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace cl_move_group_interface
{
CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(std::optional<std::string> tipLink)
: tipLink_(tipLink), markersInitialized_(false)
{
}

CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(
  const std::vector<geometry_msgs::msg::PoseStamped> & endEffectorTrajectory,
  std::optional<std::string> tipLink)
: tipLink_(tipLink), endEffectorTrajectory_(endEffectorTrajectory), markersInitialized_(false)

{
}

void CbMoveEndEffectorTrajectory::initializeROS()
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] initializing ros");

  auto nh = this->getNode();
  markersPub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 1);
  iksrv_ = nh->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
}

ComputeJointTrajectoryErrorCode CbMoveEndEffectorTrajectory::computeJointSpaceTrajectory(
  moveit_msgs::msg::RobotTrajectory & computedJointTrajectory)
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] getting current state.. waiting");

  // get current robot state
  auto currentState = movegroupClient_->moveGroupClientInterface->getCurrentState(100);

  // get the IK client
  auto groupname = movegroupClient_->moveGroupClientInterface->getName();

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] getting joint names");
  auto currentjointnames = currentState->getJointModelGroup(groupname)->getActiveJointModelNames();

  if (!tipLink_ || *tipLink_ == "")
  {
    tipLink_ = movegroupClient_->moveGroupClientInterface->getEndEffectorLink();
  }

  std::vector<double> jointPositions;
  currentState->copyJointGroupPositions(groupname, jointPositions);

  std::vector<std::vector<double>> trajectory;
  std::vector<rclcpp::Duration> trajectoryTimeStamps;

  trajectory.push_back(jointPositions);
  trajectoryTimeStamps.push_back(rclcpp::Duration(0s));

  auto & first = endEffectorTrajectory_.front();
  rclcpp::Time referenceTime(first.header.stamp);

  std::vector<int> discontinuityIndexes;

  int ikAttempts = 4;
  for (size_t k = 0; k < this->endEffectorTrajectory_.size(); k++)
  {
    auto & pose = this->endEffectorTrajectory_[k];
    auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    //req.ik_request.attempts = 20;

    req->ik_request.ik_link_name = *tipLink_;
    req->ik_request.robot_state.joint_state.name = currentjointnames;
    req->ik_request.robot_state.joint_state.position = jointPositions;

    req->ik_request.group_name = groupname;
    req->ik_request.avoid_collisions = true;

    //pose.header.stamp = getNode()->now();
    req->ik_request.pose_stamped = pose;

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] IK request: " << k << " " << *req);

    auto resfut = iksrv_->async_send_request(req);

    auto status = resfut.wait_for(3s);
    if (status == std::future_status::ready)
    {
      //if (rclcpp::spin_until_future_complete(getNode(), resfut) == rclcpp::FutureReturnCode::SUCCESS)
      //{
      auto & prevtrajpoint = trajectory.back();
      //jointPositions.clear();

      auto res = resfut.get();
      std::stringstream ss;
      for (size_t j = 0; j < res->solution.joint_state.position.size(); j++)
      {
        auto & jointname = res->solution.joint_state.name[j];
        auto it = std::find(currentjointnames.begin(), currentjointnames.end(), jointname);
        if (it != currentjointnames.end())
        {
          int index = std::distance(currentjointnames.begin(), it);
          jointPositions[index] = res->solution.joint_state.position[j];
          ss << jointname << "(" << index << "): " << jointPositions[index] << std::endl;
        }
      }

      // continuity check
      size_t jointindex = 0;
      int discontinuityJointIndex = -1;
      double discontinuityDeltaJointIndex = -1;
      double deltajoint;

      bool check = k > 0 || !allowInitialTrajectoryStateJointDiscontinuity_ ||
                   (allowInitialTrajectoryStateJointDiscontinuity_ &&
                    !(*allowInitialTrajectoryStateJointDiscontinuity_));
      if (check)
      {
        for (jointindex = 0; jointindex < jointPositions.size(); jointindex++)
        {
          deltajoint = jointPositions[jointindex] - prevtrajpoint[jointindex];

          if (fabs(deltajoint) > 0.3 /*2.5 deg*/)
          {
            discontinuityDeltaJointIndex = deltajoint;
            discontinuityJointIndex = jointindex;
          }
        }
      }

      if (ikAttempts > 0 && discontinuityJointIndex != -1)
      {
        k--;
        ikAttempts--;
        continue;
      }
      else
      {
        bool discontinuity = false;
        if (ikAttempts == 0)
        {
          discontinuityIndexes.push_back(k);
          discontinuity = true;
        }

        ikAttempts = 4;

        if (discontinuity && discontinuityJointIndex != -1)
        {
          // show a message and stop the trajectory generation && jointindex!= 7 || fabs(deltajoint) > 0.1 /*2.5 deg*/  && jointindex== 7
          std::stringstream ss;
          ss << "Traj[" << k << "/" << endEffectorTrajectory_.size() << "] "
             << currentjointnames[discontinuityJointIndex]
             << " IK discontinuity : " << discontinuityDeltaJointIndex << std::endl
             << "prev joint value: " << prevtrajpoint[discontinuityJointIndex] << std::endl
             << "current joint value: " << jointPositions[discontinuityJointIndex] << std::endl;

          ss << std::endl;
          for (size_t ji = 0; ji < jointPositions.size(); ji++)
          {
            ss << currentjointnames[ji] << ": " << jointPositions[ji] << std::endl;
          }

          for (size_t kindex = 0; kindex < trajectory.size(); kindex++)
          {
            ss << "[" << kindex << "]: " << trajectory[kindex][discontinuityJointIndex]
               << std::endl;
          }

          if (k == 0)
          {
            ss << "This is the first posture of the trajectory. Maybe the robot initial posture is "
                  "not coincident to the initial posture of the generated joint trajectory."
               << std::endl;
          }

          RCLCPP_ERROR_STREAM(getLogger(), ss.str());

          trajectory.push_back(jointPositions);
          rclcpp::Duration durationFromStart = rclcpp::Time(pose.header.stamp) - referenceTime;
          trajectoryTimeStamps.push_back(durationFromStart);

          continue;
        }
        else
        {
          trajectory.push_back(jointPositions);
          rclcpp::Duration durationFromStart = rclcpp::Time(pose.header.stamp) - referenceTime;
          trajectoryTimeStamps.push_back(durationFromStart);

          RCLCPP_DEBUG_STREAM(getLogger(), "IK solution: " << res->solution.joint_state);
          RCLCPP_DEBUG_STREAM(getLogger(), "trajpoint: " << std::endl << ss.str());
        }
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(getLogger(), "[" << getName() << "] wrong IK call");
    }
  }

  // interpolate speeds?

  // interpolate accelerations?

  // get current robot state
  // fill plan message
  // computedMotionPlan.start_state_.joint_state.name = currentjointnames;
  // computedMotionPlan.start_state_.joint_state.position = trajectory.front();
  // computedMotionPlan.trajectory_.joint_trajectory.joint_names = currentjointnames;

  computedJointTrajectory.joint_trajectory.joint_names = currentjointnames;
  int i = 0;
  for (auto & p : trajectory)
  {
    if (
      i ==
      0)  // not copy the current state in the trajectory (used to solve discontinuity in other behaviors)
    {
      i++;
      continue;
    }

    trajectory_msgs::msg::JointTrajectoryPoint jp;
    jp.positions = p;
    jp.time_from_start = trajectoryTimeStamps[i];  //rclcpp::Duration(t);
    computedJointTrajectory.joint_trajectory.points.push_back(jp);
    i++;
  }

  if (discontinuityIndexes.size())
  {
    if (discontinuityIndexes[0] == 0)
      return ComputeJointTrajectoryErrorCode::INCORRECT_INITIAL_STATE;
    else
      return ComputeJointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY;
  }

  return ComputeJointTrajectoryErrorCode::SUCCESS;
}

void CbMoveEndEffectorTrajectory::executeJointSpaceTrajectory(
  const moveit_msgs::msg::RobotTrajectory & computedJointTrajectory)
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Executing joint trajectory");
  // call execute
  auto executionResult =
    this->movegroupClient_->moveGroupClientInterface->execute(computedJointTrajectory);

  if (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] motion execution succeeded");
    movegroupClient_->postEventMotionExecutionSucceded();
    this->postSuccessEvent();
  }
  else
  {
    this->postMotionExecutionFailureEvents();
    this->postFailureEvent();
  }
}

void CbMoveEndEffectorTrajectory::onEntry()
{
  this->requiresClient(movegroupClient_);

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Generating end effector trajectory");

  this->generateTrajectory();

  if (this->endEffectorTrajectory_.size() == 0)
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << smacc2::demangleSymbol(typeid(*this).name())
                       << "] No points in the trajectory. Skipping behavior.");
    return;
  }

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Creating markers.");

  this->createMarkers();
  markersInitialized_ = true;
  moveit_msgs::msg::RobotTrajectory computedTrajectory;

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Computing joint space trajectory.");

  auto errorcode = computeJointSpaceTrajectory(computedTrajectory);

  bool trajectoryGenerationSuccess = errorcode == ComputeJointTrajectoryErrorCode::SUCCESS;

  CpTrajectoryHistory * trajectoryHistory;
  this->requiresComponent(trajectoryHistory);

  if (!trajectoryGenerationSuccess)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Incorrect trajectory. Posting failure event.");
    if (trajectoryHistory != nullptr)
    {
      moveit_msgs::msg::MoveItErrorCodes error;
      error.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      trajectoryHistory->pushTrajectory(this->getName(), computedTrajectory, error);
    }

    movegroupClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();

    if (errorcode == ComputeJointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY)
    {
      this->postJointDiscontinuityEvent(computedTrajectory);
    }
    else if (errorcode == ComputeJointTrajectoryErrorCode::INCORRECT_INITIAL_STATE)
    {
      this->postIncorrectInitialStateEvent(computedTrajectory);
    }
    return;
  }
  else
  {
    if (trajectoryHistory != nullptr)
    {
      moveit_msgs::msg::MoveItErrorCodes error;
      error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      trajectoryHistory->pushTrajectory(this->getName(), computedTrajectory, error);
    }

    this->executeJointSpaceTrajectory(computedTrajectory);
  }

  // handle finishing events
}  // namespace cl_move_group_interface

void CbMoveEndEffectorTrajectory::update()
{
  if (markersInitialized_)
  {
    std::lock_guard<std::mutex> guard(m_mutex_);
    markersPub_->publish(beahiorMarkers_);
  }
}

void CbMoveEndEffectorTrajectory::onExit()
{
  markersInitialized_ = false;

  if (autocleanmarkers)
  {
    std::lock_guard<std::mutex> guard(m_mutex_);
    for (auto & marker : this->beahiorMarkers_.markers)
    {
      marker.header.stamp = getNode()->now();
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    markersPub_->publish(beahiorMarkers_);
  }
}

void CbMoveEndEffectorTrajectory::createMarkers()
{
  tf2::Transform localdirection;
  localdirection.setIdentity();
  localdirection.setOrigin(tf2::Vector3(0.05, 0, 0));
  auto frameid = this->endEffectorTrajectory_.front().header.frame_id;

  for (auto & pose : this->endEffectorTrajectory_)
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
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;

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

void CbMoveEndEffectorTrajectory::generateTrajectory()
{
  // bypass current trajectory, overridden in derived classes
  // this->endEffectorTrajectory_ = ...
}

void CbMoveEndEffectorTrajectory::getCurrentEndEffectorPose(
  std::string globalFrame, tf2::Stamped<tf2::Transform> & currentEndEffectorTransform)
{
  tf2_ros::Buffer tfBuffer(getNode()->get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);

  try
  {
    if (!tipLink_ || *tipLink_ == "")
    {
      tipLink_ = this->movegroupClient_->moveGroupClientInterface->getEndEffectorLink();
    }

    tf2::fromMsg(
      tfBuffer.lookupTransform(globalFrame, *tipLink_, rclcpp::Time(0), rclcpp::Duration(10s)),
      currentEndEffectorTransform);

    //tfListener.lookupTransform(globalFrame, *tipLink_, rclcpp::Time(0), currentEndEffectorTransform);

    // we define here the global frame as the pivot frame id
    // tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, rclcpp::Time(0), rclcpp::Duration(10));
    // tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, rclcpp::Time(0), globalBaseLink);
  }
  catch (const std::exception & e)
  {
    std::cerr << e.what() << std::endl;
  }
}
}  // namespace cl_move_group_interface
