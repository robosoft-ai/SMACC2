/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/cl_movegroup.h>
namespace cl_move_group_interface
{

ClMoveGroup::ClMoveGroup(std::string groupName)
    : moveGroupClientInterface(groupName)
{
    ros::WallDuration(10.0).sleep();
}

ClMoveGroup::~ClMoveGroup()
{
}

void ClMoveGroup::postEventMotionExecutionSucceded()
{
    RCLCPP_INFO(nh_->get_logger(), "[ClMoveGroup] Post Motion Success Event");
    postEventMotionExecutionSucceded_();
}

void ClMoveGroup::postEventMotionExecutionFailed()
{
    RCLCPP_INFO(nh_->get_logger(), "[ClMoveGroup] Post Motion Failure Event");
    postEventMotionExecutionFailed_();
}

} // namespace cl_move_group_interface
