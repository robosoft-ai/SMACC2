/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <tf2_ros/buffer.h>

#include "cb_move_base_client_behavior_base.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cl_move_base_z
{
enum class SpiningPlanner
{
  Default,
  PureSpinning,
  Forward
};

class CbAbsoluteRotate : public CbMoveBaseClientBehaviorBase
{
public:
  std::shared_ptr<tf2_ros::Buffer> listener;

  std::optional<float> absoluteGoalAngleDegree;
  std::optional<float> yawGoalTolerance;
  std::optional<float> maxVelTheta;  // if not defined, default parameter server
  std::optional<SpiningPlanner> spinningPlanner;

  CbAbsoluteRotate();

  CbAbsoluteRotate(float absoluteGoalAngleDegree, float yawGoalTolerance = -1);

  virtual void onEntry() override;
  virtual void onExit() override;

private:
  void updateTemporalBehaviorParameters(bool undo);
  float oldYawTolerance;
  float oldMaxVelTheta;
  float oldMinVelTheta;
};
}  // namespace cl_move_base_z
