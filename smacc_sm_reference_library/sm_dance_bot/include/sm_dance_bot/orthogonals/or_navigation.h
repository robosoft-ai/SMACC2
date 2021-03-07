#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_navigator.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.h>

namespace sm_dance_bot
{

using namespace cl_move_base_z;
using namespace std::chrono_literals;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        auto movebaseClient = this->createClient<ClMoveBaseZ>();
        
        // create pose component
        movebaseClient->createComponent<cl_move_base_z::Pose>();

        // create planner switcher
        movebaseClient->createComponent<PlannerSwitcher>();

        // create goal checker switcher
        movebaseClient->createComponent<cl_move_base_z::GoalCheckerSwitcher>();

        // create odom tracker
        movebaseClient->createComponent<cl_move_base_z::odom_tracker::OdomTracker>();
        
        // create waypoints navigator component
        auto waypointsNavigator = movebaseClient->createComponent<WaypointNavigator>();
        loadWaypointsFromYaml(waypointsNavigator);

        // change this to skip some points of the yaml file, default = 0
        waypointsNavigator->currentWaypoint_ = 3;
    }

    void loadWaypointsFromYaml(WaypointNavigator *waypointsNavigator)
    {
        // if it is the first time and the waypoints navigator is not configured
        std::string planfilepath;
        
        RCLCPP_INFO_STREAM(getNode()->get_logger(), "Reasing parameter file from node: " << getNode()->get_name());
        getNode()->declare_parameter("waypoints_plan");
        if (getNode()->get_parameter("waypoints_plan", planfilepath))
        {
            waypointsNavigator->loadWayPointsFromFile(planfilepath);
            RCLCPP_INFO(getNode()->get_logger(), "waypoints plan: %s", planfilepath.c_str());
        }
        else
        {
            RCLCPP_ERROR(getNode()->get_logger(), "waypoints plan file not found: NONE");
        }
    }
};
} // namespace sm_dance_bot