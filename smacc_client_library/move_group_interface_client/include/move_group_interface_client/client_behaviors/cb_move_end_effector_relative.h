/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>

namespace cl_move_group_interface
{
class CbMoveEndEffectorRelative : public smacc::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::msg::Transform transform_;

  boost::optional<std::string> group_;

  CbMoveEndEffectorRelative();

  CbMoveEndEffectorRelative(geometry_msgs::msg::Transform transform);

  virtual void onEntry() override;

  virtual void onExit() override;

protected:
  void moveRelative(moveit::planning_interface::MoveGroupInterface &moveGroupinterface,
                    geometry_msgs::msg::Transform &transformOffset);

  ClMoveGroup *movegroupClient_;
};

}  // namespace cl_move_group_interface