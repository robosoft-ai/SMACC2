/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <future>
namespace cl_move_group_interface
{
class CbMoveEndEffector : public smacc::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::msgs::PoseStamped targetPose;
  std::string tip_link_;
  boost::optional<std::string> group_;

  CbMoveEndEffector();
  CbMoveEndEffector(geometry_msgs::msgs::PoseStamped target_pose, std::string tip_link = "");

  virtual void onEntry() override;

protected:
  bool moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                          geometry_msgs::msgs::PoseStamped &targetObjectPose);

  ClMoveGroup *movegroupClient_;
};
}  // namespace cl_move_group_interface