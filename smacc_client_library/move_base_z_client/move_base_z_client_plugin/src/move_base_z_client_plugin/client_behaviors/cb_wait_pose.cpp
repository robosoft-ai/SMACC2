/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/client_behaviors/cb_wait_pose.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <move_base_z_client_plugin/common.h>
#include <rclcpp/parameter_client.hpp>

namespace cl_move_base_z
{
CbWaitPose::CbWaitPose() {}

CbWaitPose::~CbWaitPose() {}

void CbWaitPose::onEntry()
{
  auto pose = this->moveBaseClient_->getComponent<Pose>();
  try
  {
    pose->waitTransformUpdate(rclcpp::Rate(20));
    auto posemsg = pose->toPoseMsg();
    RCLCPP_INFO_STREAM(getLogger(), "[CbWaitPose] pose arrived: " << std::endl << posemsg);
  }
  catch (std::exception & ex)
  {
    RCLCPP_INFO(getLogger(), "[CbWaitPose] error getting the robot pose");
    this->postFailureEvent();
    return;
  }

  RCLCPP_INFO(getLogger(), "[CbWaitPose] pose received");
  this->postSuccessEvent();
}
}  // namespace cl_move_base_z