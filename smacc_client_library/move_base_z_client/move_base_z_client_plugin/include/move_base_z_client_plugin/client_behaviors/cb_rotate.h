/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <tf2_ros/buffer.h>

#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{
class CbRotate : public CbMoveBaseClientBehaviorBase
{
public:
  std::optional<float> rotateDegree;

  std::optional<std::string> goalChecker_;

  CbRotate();

  CbRotate(float rotate_degree);

  virtual void onEntry() override;

private:
  std::shared_ptr<tf2_ros::Buffer> listener;
};
}  // namespace cl_move_base_z
