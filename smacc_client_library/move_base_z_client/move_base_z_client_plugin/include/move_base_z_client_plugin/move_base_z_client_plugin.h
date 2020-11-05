/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_action_client_base.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
//#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

namespace cl_move_base_z
{
class WaypointNavigator;

class ClMoveBaseZ : public smacc::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>
{
    typedef SmaccActionClientBase<nav2_msgs::action::NavigateToPose> Base;
    
public:
    typedef Base::WrappedResult WrappedResult;
    ClMoveBaseZ(std::string moveBaseName="/move_base");

    virtual ~ClMoveBaseZ();

    virtual void initialize() override;

    virtual std::string getName() const override;
};

} // namespace smacc