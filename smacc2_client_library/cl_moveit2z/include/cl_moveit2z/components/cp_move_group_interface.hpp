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

#include <smacc2/smacc.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_moveit2z
{
template <typename TSource, typename TOrthogonal>
struct EvMoveGroupMotionExecutionSucceeded
: sc::event<EvMoveGroupMotionExecutionSucceeded<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvMoveGroupMotionExecutionFailed
: sc::event<EvMoveGroupMotionExecutionFailed<TSource, TOrthogonal>>
{
};

class CpMoveGroupInterface : public smacc2::ISmaccComponent
{
public:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupClientInterface;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planningSceneInterface;

  explicit CpMoveGroupInterface(
    const moveit::planning_interface::MoveGroupInterface::Options & options)
  : options_(options)
  {
  }

  inline void onInitialize() override
  {
    moveGroupClientInterface =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(getNode(), options_);
    planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(
      options_.move_group_namespace);
    RCLCPP_INFO(getLogger(), "[CpMoveGroupInterface] MoveGroupInterface initialized");
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    postEventMotionExecutionSucceeded_ = [=]()
    {
      this->onSucceeded_();
      this->postEvent<EvMoveGroupMotionExecutionSucceeded<TSourceObject, TOrthogonal>>();
    };

    postEventMotionExecutionFailed_ = [=]()
    {
      this->onFailed_();
      this->postEvent<EvMoveGroupMotionExecutionFailed<TSourceObject, TOrthogonal>>();
    };
  }

  inline void postEventMotionExecutionSucceeded()
  {
    RCLCPP_INFO(getLogger(), "[CpMoveGroupInterface] Post Motion Success Event");
    postEventMotionExecutionSucceeded_();
  }

  inline void postEventMotionExecutionFailed()
  {
    RCLCPP_INFO(getLogger(), "[CpMoveGroupInterface] Post Motion Failure Event");
    postEventMotionExecutionFailed_();
  }

  template <typename TCallback, typename T>
  smacc2::SmaccSignalConnection onMotionExecutionSucceeded(TCallback callback, T * object)
  {
    return this->getStateMachine()->createSignalConnection(onSucceeded_, callback, object);
  }

  template <typename TCallback, typename T>
  smacc2::SmaccSignalConnection onMotionExecutionFailed(TCallback callback, T * object)
  {
    return this->getStateMachine()->createSignalConnection(onFailed_, callback, object);
  }

private:
  std::function<void()> postEventMotionExecutionSucceeded_;
  std::function<void()> postEventMotionExecutionFailed_;

  smacc2::SmaccSignal<void()> onSucceeded_;
  smacc2::SmaccSignal<void()> onFailed_;

  moveit::planning_interface::MoveGroupInterface::Options options_;
};

}  // namespace cl_moveit2z
