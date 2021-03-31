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
class CbMoveCartesianRelative : public smacc::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::msg::Vector3 offset_;

  boost::optional<double> scalingFactor_;

  boost::optional<std::string> group_;

  CbMoveCartesianRelative();

  CbMoveCartesianRelative(geometry_msgs::msg::Vector3 offset);

  virtual void onEntry() override;

  virtual void onExit() override;

  void moveRelativeCartesian(moveit::planning_interface::MoveGroupInterface *movegroupClient,
                             geometry_msgs::msg::Vector3 &offset);

  public:
    ClMoveGroup *moveGroupSmaccClient_;
};
}  // namespace cl_move_group_interface