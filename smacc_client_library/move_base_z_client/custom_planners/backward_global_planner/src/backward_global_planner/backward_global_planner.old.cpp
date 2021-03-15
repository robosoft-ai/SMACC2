/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <backward_global_planner/backward_global_planner.h>
#include <fstream>
#include <streambuf>

#include <visualization_msgs/msg/marker_array.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>
#include <move_base_z_planners_common/move_base_z_client_tools.h>
          
namespace cl_move_base_z
{
namespace backward_global_planner
{
/**
******************************************************************************************************************
* Constructor()
******************************************************************************************************************
*/
BackwardGlobalPlanner::BackwardGlobalPlanner()
{
    skip_straight_motion_distance_ = 0.2;
}

BackwardGlobalPlanner::~BackwardGlobalPlanner()
{
    //clear "rviz"- publish empty path
    nav_msgs::msg::Path planMsg;
    planMsg.header.stamp = nh_->now();
    planPub_->publish(planMsg);
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/

void  BackwardGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    //RCLCPP_INFO_NAMED(nh_->get_logger(), "Backwards", "BackwardGlobalPlanner initialize");
    nh_ = parent.lock();
    name_ = name;
    costmap_ros_ = costmap_ros;
    //RCLCPP_WARN_NAMED(nh_->get_logger(), "Backwards", "initializating global planner, costmap address: %ld", (long)costmap_ros);

    planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("backward_planner/global_plan", 1);
    markersPub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("backward_planner/markers", 1);
}

void BackwardGlobalPlanner::cleanup()
{

}

void BackwardGlobalPlanner::activate()
{
    RCLCPP_INFO_STREAM(nh_->get_logger(),"activating planner BackwardGlobalPlanner");
    planPub_->on_activate();
    markersPub_->on_activate();
}

void BackwardGlobalPlanner::deactivate() 
{
    RCLCPP_INFO_STREAM(nh_->get_logger(),"deactivating planner BackwardGlobalPlanner");
    planPub_->on_deactivate();
    markersPub_->on_deactivate();
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::publishGoalMarker(const geometry_msgs::msg::Pose &pose, double r, double g, double b)
{
    double phi = tf2::getYaw(pose.orientation);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->costmap_ros_->getGlobalFrameID();
    marker.header.stamp = nh_->now();
    marker.ns = "my_namespace2";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    geometry_msgs::msg::Point start, end;
    start.x = pose.position.x;
    start.y = pose.position.y;

    end.x = pose.position.x + 0.5 * cos(phi);
    end.y = pose.position.y + 0.5 * sin(phi);

    marker.points.push_back(start);
    marker.points.push_back(end);

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.push_back(marker);

    markersPub_->publish(ma);
}

/**
******************************************************************************************************************
* defaultBackwardPath()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::createDefaultBackwardPath(const geometry_msgs::msg::PoseStamped &start,
                                                      const geometry_msgs::msg::PoseStamped &goal, std::vector<geometry_msgs::msg::PoseStamped> &plan)
{
    auto q = start.pose.orientation;

    geometry_msgs::msg::PoseStamped pose;
    pose = start;
    
    double dx = start.pose.position.x - goal.pose.position.x;
    double dy = start.pose.position.y - goal.pose.position.y;

    double lenght = sqrt(dx * dx + dy * dy);

    geometry_msgs::msg::PoseStamped prevState;
    if (lenght > skip_straight_motion_distance_)
    {
        // skip initial pure spinning and initial straight motion
        //RCLCPP_INFO(getNode()->get_logger(),"1 - heading to goal position pure spinning");
        double heading_direction = atan2(dy, dx);
        double startyaw = tf2::getYaw(q);
        double offset = angles::shortest_angular_distance(startyaw, heading_direction);
        heading_direction = startyaw + offset;

        prevState = cl_move_base_z::makePureSpinningSubPlan(start, heading_direction, plan, puresSpinningRadStep_);
        //RCLCPP_INFO(getNode()->get_logger(),"2 - going forward keep orientation pure straight");

        prevState = cl_move_base_z::makePureStraightSubPlan(prevState, goal.pose.position, lenght, plan);
    }
    else
    {
        prevState = start;
    }

    RCLCPP_WARN_STREAM( nh_->get_logger(), "[BackwardGlobalPlanner ] backward global plan size:  " <<plan.size());
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
nav_msgs::msg::Path BackwardGlobalPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    //RCLCPP_WARN("Backwards", "Backwards global planner: Generating global plan ");
    //RCLCPP_WARN("Backwards", "Clearing...");

    std::vector<geometry_msgs::msg::PoseStamped> plan;

    this->createDefaultBackwardPath(start, goal, plan);
    //this->createPureSpiningAndStragihtLineBackwardPath(start, goal, plan);

    //RCLCPP_INFO_STREAM(" start - " << start);
    //RCLCPP_INFO_STREAM(" end - " << goal.pose.position);

    //RCLCPP_INFO_INFO("3 - heading to goal orientation");
    //double goalOrientation = angles::normalize_angle(tf2::getYaw(goal.pose.orientation));
    //cl_move_base_z::makePureSpinningSubPlan(prevState,goalOrientation,plan);

    //RCLCPP_WARN_STREAM(getNode()->get_logger(), "MAKE PLAN INVOKED, plan size:"<< plan.size());
    publishGoalMarker(goal.pose, 1.0, 0, 1.0);

    nav_msgs::msg::Path planMsg;
    planMsg.poses = plan;
    planMsg.header.frame_id = this->costmap_ros_->getGlobalFrameID();

        // check plan rejection
    bool acceptedGlobalPlan = true;

    // static const unsigned char NO_INFORMATION = 255;
    // static const unsigned char LETHAL_OBSTACLE = 254;
    // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    // static const unsigned char FREE_SPACE = 0;

    auto costmap2d = this->costmap_ros_->getCostmap();
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
    
    RCLCPP_WARN_STREAM(nh_->get_logger(), "backward global plan publishing path. poses count: " << planMsg.poses.size());
    planPub_->publish(planMsg);

    return planMsg;
}

} // namespace backward_global_planner
} // namespace cl_move_base_z

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::backward_global_planner::BackwardGlobalPlanner, nav2_core::GlobalPlanner)
