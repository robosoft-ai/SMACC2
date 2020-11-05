/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/client_bases/smacc_action_client.h>
#include <smacc/component.h>

// #include <dynamic_reconfigure/Config.h>
// #include <dynamic_reconfigure/DoubleParameter.h>
// #include <dynamic_reconfigure/Reconfigure.h>
#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace cl_move_base_z
{
class PlannerSwitcher : public smacc::ISmaccComponent
{
public:
  PlannerSwitcher();
  void setBackwardPlanner();
  void setUndoPathBackwardPlanner();

  void setForwardPlanner();
  void setPureSpinningPlanner();

  virtual void onInitialize() override;

  // sets ROS defaults local and global planners
  void setDefaultPlanners();

private:
  std::string desired_global_planner_;
  std::string desired_local_planner_;
  // ros::Subscriber dynrecofSub_;
  bool set_planners_mode_flag;

  void updatePlanners(bool subscribecallback = true);
  // void dynreconfCallback(const dynamic_reconfigure::Config::ConstPtr& configuration_update);
};
}  // namespace cl_move_base_z
