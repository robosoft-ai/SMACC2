/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_asynchronous_client_behavior.h>
#include <move_group_interface_client/cl_movegroup.h>
#include <map>
#include <string>

namespace cl_move_group_interface
{
  //named targets are configured in the urdf file
class CbMoveNamedTarget : public smacc::SmaccAsyncClientBehavior
{
protected:
  ClMoveGroup *movegroupClient_;
  std::string namedTarget_;

public:
  CbMoveNamedTarget(std::string namedtarget);

  virtual void onEntry() override;

  virtual void onExit() override;

  std::map<std::string, double> getNamedTargetValues();
};
}  // namespace cl_move_group_interface
