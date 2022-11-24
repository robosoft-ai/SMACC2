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
#include <move_group_interface_client/cl_movegroup.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace cl_move_group_interface
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

// this is a base behavior to define any kind of parametrized family of trajectories or motions
class CbMoveEndEffectorTrajectory : public smacc2::SmaccAsyncClientBehavior,
                                    public smacc2::ISmaccUpdatable
{
public:
  // std::string tip_link_;
  std::optional<std::string> group_;

  std::optional<std::string> tipLink_;

  std::optional<bool> allowInitialTrajectoryStateJointDiscontinuity_;

  CbMoveEndEffectorTrajectory(std::optional<std::string> tipLink = std::nullopt);

  CbMoveEndEffectorTrajectory(
    const std::vector<geometry_msgs::msg::PoseStamped> & endEffectorTrajectory,
    std::optional<std::string> tipLink = std::nullopt);

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->initializeROS();

    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    postJointDiscontinuityEvent = [this](auto traj) {
      auto ev = new EvJointDiscontinuity<TSourceObject, TOrthogonal>();
      ev->trajectory = traj;
      this->postEvent(ev);
    };

    postIncorrectInitialStateEvent = [this](auto traj) {
      auto ev = new EvIncorrectInitialPosition<TSourceObject, TOrthogonal>();
      ev->trajectory = traj;
      this->postEvent(ev);
    };

    postMotionExecutionFailureEvents = [this] {
      RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postEvent<EvMoveGroupMotionExecutionFailed<TSourceObject, TOrthogonal>>();
    };
  }

  virtual void onEntry() override;

  virtual void onExit() override;

  virtual void update() override;

protected:
  ComputeJointTrajectoryErrorCode computeJointSpaceTrajectory(
    moveit_msgs::msg::RobotTrajectory & computedJointTrajectory);

  void executeJointSpaceTrajectory(
    const moveit_msgs::msg::RobotTrajectory & computedJointTrajectory);

  virtual void generateTrajectory() = 0;

  virtual void createMarkers();

  std::vector<geometry_msgs::msg::PoseStamped> endEffectorTrajectory_;

  ClMoveGroup * movegroupClient_ = nullptr;

  visualization_msgs::msg::MarkerArray beahiorMarkers_;

  void getCurrentEndEffectorPose(
    std::string globalFrame, tf2::Stamped<tf2::Transform> & currentEndEffectorTransform);

private:
  void initializeROS();

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markersPub_;

  std::atomic<bool> markersInitialized_;

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr iksrv_;

  std::mutex m_mutex_;

  std::function<void(moveit_msgs::msg::RobotTrajectory &)> postJointDiscontinuityEvent;
  std::function<void(moveit_msgs::msg::RobotTrajectory &)> postIncorrectInitialStateEvent;

  std::function<void()> postMotionExecutionFailureEvents;

  bool autocleanmarkers = false;
};
}  // namespace cl_move_group_interface
