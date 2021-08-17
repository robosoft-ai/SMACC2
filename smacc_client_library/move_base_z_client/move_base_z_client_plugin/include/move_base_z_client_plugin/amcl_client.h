/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <smacc/smacc.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
//#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

namespace cl_move_base_z
{
class Amcl : public smacc::ISmaccClient
{
public:
  Amcl();
  virtual ~Amcl();
  std::string getName() const override;
};

}  // namespace cl_move_base_z
