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
class CbUndoPathBackwards : public CbMoveBaseClientBehaviorBase
{
public:
  std::optional<std::string> goalChecker_;

  virtual void onEntry() override;

  virtual void onExit() override;

private:
  std::shared_ptr<tf2_ros::Buffer> listener;
};
}  // namespace cl_move_base_z
