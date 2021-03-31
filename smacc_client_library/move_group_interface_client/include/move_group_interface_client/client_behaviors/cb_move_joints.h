/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <map>
#include <string>

namespace cl_move_group_interface
{
class CbMoveJoints : public smacc::SmaccAsyncClientBehavior
{
public:
  boost::optional<double> scalingFactor_;
  std::map<std::string, double> jointValueTarget_;
  boost::optional<std::string> group_;

  CbMoveJoints();
  CbMoveJoints(const std::map<std::string, double> &jointValueTarget);
  virtual void onEntry() override;
  virtual void onExit() override;

protected:
  void moveJoints(moveit::planning_interface::MoveGroupInterface &moveGroupInterface);
  ClMoveGroup *movegroupClient_;
  };
}  // namespace cl_move_group_interface
