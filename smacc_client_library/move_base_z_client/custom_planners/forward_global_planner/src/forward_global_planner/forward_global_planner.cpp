/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <forward_global_planner/forward_global_planner.h>
#include <move_base_z_planners_common/move_base_z_client_tools.h>

#include <fstream>
#include <streambuf>
#include <nav_msgs/msg/path.h>

//#include <tf/tf.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace cl_move_base_z
{
namespace forward_global_planner
{
ForwardGlobalPlanner::ForwardGlobalPlanner()
    //: nh_("~/ForwardGlobalPlanner")
{
    skip_straight_motion_distance_ = 0.2; //meters
    puresSpinningRadStep_ = 1000;         // rads
}

ForwardGlobalPlanner::~ForwardGlobalPlanner() 
{

}


void ForwardGlobalPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    nh_ = parent;
    name_ = name;

    RCLCPP_INFO(nh_->get_logger(),"[Forward Global Planner] initializing, name: %s", name_);
    planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
    skip_straight_motion_distance_ = 0.2; //meters
    puresSpinningRadStep_ = 1000;         // rads
    this->costmap_ros_ = costmap_ros;
}

void ForwardGlobalPlanner::cleanup()
{

}

void ForwardGlobalPlanner::activate()
{

}

void ForwardGlobalPlanner::deactivate() 
{

}

nav_msgs::msg::Path ForwardGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
    RCLCPP_INFO(nh_->get_logger(),"[Forward Global Planner] planning");
    nav_msgs::msg::Path planMsg;
    std::vector<geometry_msgs::msg::PoseStamped> plan;
    
    //ROS_WARN_STREAM("Forward global plan goal: " << goal);

    //three stages: 1 - heading to goal position, 2 - going forward keep orientation, 3 - heading to goal orientation

    // 1 - heading to goal position
    // orientation direction

    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;

    double lenght = sqrt(dx * dx + dy * dy);

    geometry_msgs::msg::PoseStamped prevState;
    if (lenght > skip_straight_motion_distance_)
    {
        // skip initial pure spinning and initial straight motion
        //RCLCPP_INFO(nh_->get_logger(),"1 - heading to goal position pure spinning");
        double heading_direction = atan2(dy, dx);
        prevState = cl_move_base_z::makePureSpinningSubPlan(start, heading_direction, plan, puresSpinningRadStep_);

        //RCLCPP_INFO(nh_->get_logger(),"2 - going forward keep orientation pure straight");
        prevState = cl_move_base_z::makePureStraightSubPlan(prevState, goal.pose.position, lenght, plan);
    }
    else
    {
        prevState = start;
    }

    //RCLCPP_INFO(nh_->get_logger(),"3 - heading to goal orientation");
    double goalOrientation = angles::normalize_angle(tf2::getYaw(goal.pose.orientation));
    cl_move_base_z::makePureSpinningSubPlan(prevState, goalOrientation, plan, puresSpinningRadStep_);

    planMsg.poses = plan;
    planMsg.header.stamp = this->nh_->now();
    
    planMsg.header.frame_id = this->costmap_ros_->getGlobalFrameID();

    // check plan rejection
    bool acceptedGlobalPlan = true;

    // static const unsigned char NO_INFORMATION = 255;
    // static const unsigned char LETHAL_OBSTACLE = 254;
    // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    // static const unsigned char FREE_SPACE = 0;

    nav2_costmap_2d::Costmap2D* costmap2d = this->costmap_ros_->getCostmap();
    for (auto &p : plan)
    {
        unsigned int mx, my;
        costmap2d->worldToMap(p.pose.position.x, p.pose.position.y, mx, my);
        auto cost = costmap2d->getCost(mx, my);

        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            acceptedGlobalPlan = false;
            break;
        }
    }

    if (acceptedGlobalPlan)
    {
        planPub_->publish(planMsg);
        //ROS_INFO_STREAM("global forward plan: " << planMsg);
        return planMsg;
    }
    else
    {
        planMsg.poses.clear();
        return planMsg;
    }
}

} // namespace forward_global_planner
} // namespace cl_move_base_z

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::forward_global_planner::ForwardGlobalPlanner, nav2_core::GlobalPlanner);