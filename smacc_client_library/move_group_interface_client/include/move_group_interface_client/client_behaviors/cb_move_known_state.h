/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include "cb_move_joints.h"
#include <map>
#include <string>

namespace cl_move_group_interface
{
class CbMoveKnownState : public CbMoveJoints
{
public:
  CbMoveKnownState(std::string pkg, std::string config_path);
  virtual ~CbMoveKnownState();

  private:
  static std::map<std::string, double> loadJointStatesFromFile(std::string pkg, std::string filepath);
};
}  // namespace cl_move_group_interface
