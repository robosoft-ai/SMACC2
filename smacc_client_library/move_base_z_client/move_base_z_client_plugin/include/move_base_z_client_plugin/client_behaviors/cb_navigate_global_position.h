/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{
class CbNavigateGlobalPosition : public CbMoveBaseClientBehaviorBase
{
public:
  std::optional<geometry_msgs::msg::Point> goalPosition;
  std::optional<float> goalYaw;
  std::optional<float> yawTolerance;
  std::optional<float> yawToleranceX;
  std::optional<float> yawToleranceY;

  std::optional<std::string> goalChecker_;

  CbNavigateGlobalPosition();

  CbNavigateGlobalPosition(float x, float y, float yaw /*radians*/);

  void setGoal(const geometry_msgs::msg::Pose &pose);

  virtual void onEntry();

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  virtual void onExit() override;

  // auxiliar function that defines the motion that is requested to the move_base action server
  void execute();

private:
  void readStartPoseFromParameterServer(ClMoveBaseZ::Goal &goal);
};
}  // namespace cl_move_base_z
