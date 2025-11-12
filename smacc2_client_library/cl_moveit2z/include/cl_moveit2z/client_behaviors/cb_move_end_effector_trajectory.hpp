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

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/common.hpp>
#include <cl_moveit2z/components/cp_joint_space_trajectory_planner.hpp>
#include <cl_moveit2z/components/cp_tf_listener.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <cl_moveit2z/components/cp_trajectory_history.hpp>
#include <cl_moveit2z/components/cp_trajectory_visualizer.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace cl_moveit2z
{
template <typename AsyncCB, typename Orthogonal>
struct EvJointDiscontinuity : sc::event<EvJointDiscontinuity<AsyncCB, Orthogonal>>
{
  moveit_msgs::msg::RobotTrajectory trajectory;
};

template <typename AsyncCB, typename Orthogonal>
struct EvIncorrectInitialPosition : sc::event<EvIncorrectInitialPosition<AsyncCB, Orthogonal>>
{
  moveit_msgs::msg::RobotTrajectory trajectory;
};

enum class ComputeJointTrajectoryErrorCode
{
  SUCCESS,
  INCORRECT_INITIAL_STATE,
  JOINT_TRAJECTORY_DISCONTINUITY
};

// this is a base behavior to define any kind of parameterized family of trajectories or motions
class CbMoveEndEffectorTrajectory : public smacc2::SmaccAsyncClientBehavior
{
public:
  // std::string tip_link_;
  std::optional<std::string> group_;

  std::optional<std::string> tipLink_;

  std::optional<bool> allowInitialTrajectoryStateJointDiscontinuity_;

  CbMoveEndEffectorTrajectory(std::optional<std::string> tipLink = std::nullopt) : tipLink_(tipLink)
  {
  }

  CbMoveEndEffectorTrajectory(
    const std::vector<geometry_msgs::msg::PoseStamped> & endEffectorTrajectory,
    std::optional<std::string> tipLink = std::nullopt)
  : tipLink_(tipLink), endEffectorTrajectory_(endEffectorTrajectory)
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->initializeROS();

    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    postJointDiscontinuityEvent = [this](auto traj)
    {
      auto ev = new EvJointDiscontinuity<TSourceObject, TOrthogonal>();
      ev->trajectory = traj;
      this->postEvent(ev);
    };

    postIncorrectInitialStateEvent = [this](auto traj)
    {
      auto ev = new EvIncorrectInitialPosition<TSourceObject, TOrthogonal>();
      ev->trajectory = traj;
      this->postEvent(ev);
    };

    postMotionExecutionFailureEvents = [this]
    {
      RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postEvent<EvMoveGroupMotionExecutionFailed<TSourceObject, TOrthogonal>>();
    };
  }

  virtual void onEntry() override
  {
    this->requiresClient(movegroupClient_);

    // Get optional components for visualization
    CpTrajectoryVisualizer * trajectoryVisualizer = nullptr;
    this->requiresComponent(trajectoryVisualizer, false);  // Optional component

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Generating end effector trajectory");

    this->generateTrajectory();

    if (this->endEffectorTrajectory_.size() == 0)
    {
      RCLCPP_WARN_STREAM(
        getLogger(), "[" << smacc2::demangleSymbol(typeid(*this).name())
                         << "] No points in the trajectory. Skipping behavior.");
      return;
    }

    // Use CpTrajectoryVisualizer if available, otherwise use legacy marker system
    if (trajectoryVisualizer != nullptr)
    {
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[" << getName() << "] Setting trajectory visualization using CpTrajectoryVisualizer.");
      trajectoryVisualizer->setTrajectory(this->endEffectorTrajectory_, "trajectory");
    }
    else
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName()
                         << "] Creating markers (legacy mode - consider adding "
                            "CpTrajectoryVisualizer component).");
      this->createMarkers();
    }

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
  }

  virtual void onExit() override
  {
    // Get optional components for visualization cleanup
    CpTrajectoryVisualizer * trajectoryVisualizer = nullptr;
    this->requiresComponent(trajectoryVisualizer, false);  // Optional component

    if (trajectoryVisualizer != nullptr && autocleanmarkers)
    {
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[" << getName() << "] Clearing trajectory markers via CpTrajectoryVisualizer.");
      trajectoryVisualizer->clearMarkers();
    }
    else if (autocleanmarkers)
    {
      // Legacy marker cleanup
      std::lock_guard<std::mutex> guard(m_mutex_);
      for (auto & marker : this->beahiorMarkers_.markers)
      {
        marker.header.stamp = getNode()->now();
        marker.action = visualization_msgs::msg::Marker::DELETE;
      }

      if (markersPub_)
      {
        markersPub_->publish(beahiorMarkers_);
      }
    }
  }

protected:
  ComputeJointTrajectoryErrorCode computeJointSpaceTrajectory(
    moveit_msgs::msg::RobotTrajectory & computedJointTrajectory)
  {
    // Try to use CpJointSpaceTrajectoryPlanner component (preferred)
    CpJointSpaceTrajectoryPlanner * trajectoryPlanner = nullptr;
    this->requiresComponent(trajectoryPlanner, false);  // Optional component

    if (trajectoryPlanner != nullptr)
    {
      // Use component-based trajectory planner (preferred)
      RCLCPP_INFO(
        getLogger(),
        "[CbMoveEndEffectorTrajectory] Using CpJointSpaceTrajectoryPlanner component for IK "
        "trajectory generation");

      JointTrajectoryOptions options;
      if (tipLink_ && !tipLink_->empty())
      {
        options.tipLink = *tipLink_;
      }
      if (allowInitialTrajectoryStateJointDiscontinuity_)
      {
        options.allowInitialDiscontinuity = *allowInitialTrajectoryStateJointDiscontinuity_;
      }

      auto result = trajectoryPlanner->planFromWaypoints(endEffectorTrajectory_, options);

      if (result.success)
      {
        computedJointTrajectory = result.trajectory;
        RCLCPP_INFO(
          getLogger(),
          "[CbMoveEndEffectorTrajectory] IK trajectory generation succeeded (via "
          "CpJointSpaceTrajectoryPlanner)");
        return ComputeJointTrajectoryErrorCode::SUCCESS;
      }
      else
      {
        computedJointTrajectory = result.trajectory;  // Still return partial trajectory
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveEndEffectorTrajectory] IK trajectory generation failed (via "
          "CpJointSpaceTrajectoryPlanner): %s",
          result.errorMessage.c_str());

        // Map error codes
        switch (result.errorCode)
        {
          case JointTrajectoryErrorCode::INCORRECT_INITIAL_STATE:
            return ComputeJointTrajectoryErrorCode::INCORRECT_INITIAL_STATE;
          case JointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY:
            return ComputeJointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY;
          default:
            return ComputeJointTrajectoryErrorCode::JOINT_TRAJECTORY_DISCONTINUITY;
        }
      }
    }
    else
    {
      // Fallback to legacy IK trajectory generation
      RCLCPP_WARN(
        getLogger(),
        "[CbMoveEndEffectorTrajectory] CpJointSpaceTrajectoryPlanner component not available, "
        "using legacy IK trajectory generation (consider adding CpJointSpaceTrajectoryPlanner "
        "component)");

      // LEGACY IMPLEMENTATION (keep for backward compatibility)
      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] getting current state.. waiting");

      auto currentState = movegroupClient_->moveGroupClientInterface->getCurrentState(100);
      auto groupname = movegroupClient_->moveGroupClientInterface->getName();

      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] getting joint names");
      auto currentjointnames =
        currentState->getJointModelGroup(groupname)->getActiveJointModelNames();

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

        req->ik_request.ik_link_name = *tipLink_;
        req->ik_request.robot_state.joint_state.name = currentjointnames;
        req->ik_request.robot_state.joint_state.position = jointPositions;
        req->ik_request.group_name = groupname;
        req->ik_request.avoid_collisions = true;
        req->ik_request.pose_stamped = pose;

        RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] IK request: " << k << " " << *req);

        auto resfut = iksrv_->async_send_request(req);
        auto status = resfut.wait_for(3s);

        if (status == std::future_status::ready)
        {
          auto & prevtrajpoint = trajectory.back();
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

          // Continuity check
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
              if (fabs(deltajoint) > 0.3)
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
                ss
                  << "This is the first posture of the trajectory. Maybe the robot initial posture "
                     "is "
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

      computedJointTrajectory.joint_trajectory.joint_names = currentjointnames;
      int i = 0;
      for (auto & p : trajectory)
      {
        if (i == 0)  // Skip current state
        {
          i++;
          continue;
        }

        trajectory_msgs::msg::JointTrajectoryPoint jp;
        jp.positions = p;
        jp.time_from_start = trajectoryTimeStamps[i];
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
  }

  void executeJointSpaceTrajectory(
    const moveit_msgs::msg::RobotTrajectory & computedJointTrajectory)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Executing joint trajectory");

    // Try to use CpTrajectoryExecutor component (preferred)
    CpTrajectoryExecutor * trajectoryExecutor = nullptr;
    this->requiresComponent(trajectoryExecutor, false);  // Optional component

    bool executionSuccess = false;

    if (trajectoryExecutor != nullptr)
    {
      // Use component-based trajectory executor (preferred)
      RCLCPP_INFO(
        getLogger(),
        "[CbMoveEndEffectorTrajectory] Using CpTrajectoryExecutor component for execution");

      ExecutionOptions execOptions;
      execOptions.trajectoryName = this->getName();

      auto execResult = trajectoryExecutor->execute(computedJointTrajectory, execOptions);
      executionSuccess = execResult.success;

      if (executionSuccess)
      {
        RCLCPP_INFO(
          getLogger(),
          "[CbMoveEndEffectorTrajectory] Execution succeeded (via CpTrajectoryExecutor)");
      }
      else
      {
        RCLCPP_WARN(
          getLogger(),
          "[CbMoveEndEffectorTrajectory] Execution failed (via CpTrajectoryExecutor): %s",
          execResult.errorMessage.c_str());
      }
    }
    else
    {
      // Fallback to legacy direct execution
      RCLCPP_WARN(
        getLogger(),
        "[CbMoveEndEffectorTrajectory] CpTrajectoryExecutor component not available, using legacy "
        "execution (consider adding CpTrajectoryExecutor component)");

      auto executionResult =
        this->movegroupClient_->moveGroupClientInterface->execute(computedJointTrajectory);
      executionSuccess = (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

      RCLCPP_INFO(
        getLogger(), "[CbMoveEndEffectorTrajectory] Execution %s (legacy mode)",
        executionSuccess ? "succeeded" : "failed");
    }

    // Post events
    if (executionSuccess)
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

  virtual void generateTrajectory()
  {
    // bypass current trajectory, overridden in derived classes
    // this->endEffectorTrajectory_ = ...
  }

  virtual void createMarkers()
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

  std::vector<geometry_msgs::msg::PoseStamped> endEffectorTrajectory_;

  ClMoveit2z * movegroupClient_ = nullptr;

  visualization_msgs::msg::MarkerArray beahiorMarkers_;

  void getCurrentEndEffectorPose(
    std::string globalFrame, tf2::Stamped<tf2::Transform> & currentEndEffectorTransform)
  {
    // Use CpTfListener component for transform lookups
    CpTfListener * tfListener = nullptr;
    this->requiresComponent(tfListener, false);  // Optional component

    try
    {
      if (!tipLink_ || *tipLink_ == "")
      {
        tipLink_ = this->movegroupClient_->moveGroupClientInterface->getEndEffectorLink();
      }

      if (tfListener != nullptr)
      {
        // Use component-based TF listener (preferred)
        auto transformOpt = tfListener->lookupTransform(globalFrame, *tipLink_, rclcpp::Time(0));
        if (transformOpt)
        {
          tf2::fromMsg(transformOpt.value(), currentEndEffectorTransform);
        }
        else
        {
          RCLCPP_ERROR_STREAM(
            getLogger(), "[" << getName() << "] Failed to lookup transform from " << *tipLink_
                             << " to " << globalFrame);
        }
      }
      else
      {
        // Fallback to legacy TF2 usage if component not available
        RCLCPP_WARN_STREAM(
          getLogger(), "[" << getName()
                           << "] CpTfListener component not available, using legacy TF2 (consider "
                              "adding CpTfListener component)");
        tf2_ros::Buffer tfBuffer(getNode()->get_clock());
        tf2_ros::TransformListener tfListenerLegacy(tfBuffer);

        tf2::fromMsg(
          tfBuffer.lookupTransform(globalFrame, *tipLink_, rclcpp::Time(0), rclcpp::Duration(10s)),
          currentEndEffectorTransform);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(
        getLogger(), "[" << getName() << "] Exception in getCurrentEndEffectorPose: " << e.what());
    }
  }

private:
  void initializeROS()
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] initializing ros");

    auto nh = this->getNode();

    // Only create marker publisher for legacy mode (when CpTrajectoryVisualizer is not used)
    markersPub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", rclcpp::QoS(1));

    iksrv_ = nh->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markersPub_;

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr iksrv_;

  std::mutex m_mutex_;

  std::function<void(moveit_msgs::msg::RobotTrajectory &)> postJointDiscontinuityEvent;
  std::function<void(moveit_msgs::msg::RobotTrajectory &)> postIncorrectInitialStateEvent;

  std::function<void()> postMotionExecutionFailureEvents;

  bool autocleanmarkers = false;
};
}  // namespace cl_moveit2z
