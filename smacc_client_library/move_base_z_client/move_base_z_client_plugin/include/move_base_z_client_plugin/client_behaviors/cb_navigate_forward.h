/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <optional>

#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{
class CbNavigateForward : public CbMoveBaseClientBehaviorBase
{
public:
  std::optional<float> forwardDistance;

  // just a stub to show how to use parameterless constructor
  std::optional<float> forwardSpeed;

  // this may be useful in the case you want to do a stright line with some orientation
  // relative to some global reference (trying to solve the initial orientation error
  // of the current orientation). If it is not set, the orientation of the straight line is
  // the orientation of the initial state.
  std::optional<geometry_msgs::msg::Quaternion> forceInitialOrientation;

  std::optional<std::string> goalChecker_;

  std::shared_ptr<tf2_ros::Buffer> listener;

  odom_tracker::OdomTracker *odomTracker_;

  CbNavigateForward(float forwardDistance);

  CbNavigateForward();

  virtual ~CbNavigateForward();

  virtual void onEntry() override;

  virtual void onExit() override;
};
}  // namespace cl_move_base_z