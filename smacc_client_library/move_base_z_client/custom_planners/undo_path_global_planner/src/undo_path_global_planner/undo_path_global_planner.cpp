/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <undo_path_global_planner/undo_path_global_planner.h>
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>

//register this planner as a BaseGlobalPlanner plugin

namespace cl_move_base_z
{
namespace undo_path_global_planner
{
/**
******************************************************************************************************************
* Constructor()
******************************************************************************************************************
*/
UndoPathGlobalPlanner::UndoPathGlobalPlanner()
{
    skip_straight_motion_distance_ = 0.2;
}

UndoPathGlobalPlanner::~UndoPathGlobalPlanner()
{
    //clear "rviz"- publish empty path
    nav_msgs::msg::Path planMsg;
    planMsg.header.stamp = this->nh_->now();
    planPub_->publish(planMsg);
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/

void  UndoPathGlobalPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    //ROS_INFO_NAMED("Backwards", "UndoPathGlobalPlanner initialize");
    costmap_ros_ = costmap_ros;
    //ROS_WARN_NAMED("Backwards", "initializating global planner, costmap address: %ld", (long)costmap_ros);

    rclcpp::SensorDataQoS qos;
    qos.keep_last(2);
    forwardPathSub_ = nh_->create_subscription<nav_msgs::msg::Path>("odom_tracker_path", qos, std::bind(&UndoPathGlobalPlanner::onForwardTrailMsg, this, std::placeholders::_1));

    planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("undo_path_planner/global_plan", 1);
    markersPub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("undo_path_planner/markers", 1);
}

        /**
******************************************************************************************************************
* onForwardTrailMsg()
******************************************************************************************************************
*/
void UndoPathGlobalPlanner::onForwardTrailMsg(const nav_msgs::msg::Path::SharedPtr trailMessage)
{
    lastForwardPathMsg_ = *trailMessage;
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] received backward path msg poses [" << lastForwardPathMsg_.poses.size() << "]");
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void UndoPathGlobalPlanner::publishGoalMarker(const geometry_msgs::msg::Pose &pose, double r, double g, double b)
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
* createDefaultUndoPathPlan()
******************************************************************************************************************
*/
void UndoPathGlobalPlanner::createDefaultUndoPathPlan(const geometry_msgs::msg::PoseStamped &start,
                                                        const geometry_msgs::msg::PoseStamped &goal,
                                                        std::vector<geometry_msgs::msg::PoseStamped> &plan)
{
    //ROS_WARN_NAMED("Backwards", "Iterating in last forward cord path");
    int i = lastForwardPathMsg_.poses.size() - 1;
    double linear_mindist = std::numeric_limits<double>::max();
    int mindistindex = -1;
    double startPoseAngle = tf2::getYaw(start.pose.orientation);
    geometry_msgs::msg::Pose startPositionProjected;

    // The goal of this code is finding the most convinient initial path pose.
    // first, find closest linear point to the current robot position
    // we start from the final goal, that is, the begining of the trajectory
    // (since this was the forward motion from the odom tracker)
    for (auto &p : lastForwardPathMsg_.poses /*| boost::adaptors::reversed*/)
    {
        geometry_msgs::msg::PoseStamped pose = p;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.header.stamp = nh_->now();

        double dx = pose.pose.position.x - start.pose.position.x;
        double dy = pose.pose.position.y - start.pose.position.y;

        double dist = sqrt(dx * dx + dy * dy);
        double angleOrientation = tf2::getYaw(pose.pose.orientation);
        double angleError = fabs(angles::shortest_angular_distance(angleOrientation, startPoseAngle));
        if (dist <= linear_mindist)
        {
            mindistindex = i;
            linear_mindist = dist;
            startPositionProjected = pose.pose;

            RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search, NEWBEST_LINEAR= " << i << ". error, linear: " << linear_mindist << ", angular: " << angleError);
        }
        else
        {
            RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search, skipped= " << i << ". best linear error: " << linear_mindist << ". current error, linear: " << dist << " angular: " << angleError);
        }

        i--;
    }

    double const ERROR_DISTANCE_PURE_SPINNING_FACTOR = 1.5;
    // Concept of second pass: now we only consider a pure spinning motion in this point. We want to consume some very close angular targets, (accepting a larger linear minerror of 1.5 besterror. That is, more or less in the same point).

    RCLCPP_DEBUG(nh_->get_logger(), "[UndoPathGlobalPlanner] second angular pass");
    double angularMinDist = std::numeric_limits<double>::max();
    
        if(mindistindex >=  lastForwardPathMsg_.poses.size())
            mindistindex = lastForwardPathMsg_.poses.size() -1;// workaround, something is making a out of bound exception in poses array access
    {
        for (int i = mindistindex; i >= 0; i--)
        {
            // warning this index, i refers to some inverse interpretation from the previous loop,
            // (last indexes in this path corresponds to the poses closer to our current position)
            RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] " << i << "/" << lastForwardPathMsg_.poses.size());
            geometry_msgs::msg::PoseStamped pose = lastForwardPathMsg_.poses[lastForwardPathMsg_.poses.size() - i -1];
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.header.stamp = nh_->now();

            double dx = pose.pose.position.x - start.pose.position.x;
            double dy = pose.pose.position.y - start.pose.position.y;

            double dist = sqrt(dx * dx + dy * dy);
            if (dist <= linear_mindist * ERROR_DISTANCE_PURE_SPINNING_FACTOR)
            {
                double angleOrientation = tf2::getYaw(pose.pose.orientation);
                double angleError = fabs(angles::shortest_angular_distance(angleOrientation, startPoseAngle));
                if (angleError < angularMinDist)
                {
                    angularMinDist = angleError;
                    mindistindex = i;
                    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search (angular update), NEWBEST_ANGULAR= " << i << ". error, linear: "<<  dist << "(" << linear_mindist << ")" <<", angular: " << angleError << "(" << angularMinDist << ")");
                }
                else
                {
                    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search (angular update), skipped= " << i <<". error, linear: "<<  dist << "(" << linear_mindist << ")" <<", angular: "  << angleError << "(" << angularMinDist << ")");
                }
            }
            else
            {
                RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search (angular update) not in linear range, skipped= " << i << " linear error: " << dist << "(" << linear_mindist << ")");
            }
        }
    }

    if (mindistindex != -1)
    {
        //plan.push_back(start);

        RCLCPP_WARN_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Creating the backwards plan from odom tracker path (" << lastForwardPathMsg_.poses.size() << ") poses");
        RCLCPP_WARN_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] closer point to goal i=" << mindistindex << " (linear min dist " << linear_mindist << ")");

        // copy the path at the inverse direction
        for (int i = lastForwardPathMsg_.poses.size() - 1; i >= mindistindex; i--)
        {
            auto &pose = lastForwardPathMsg_.poses[i];
            RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] adding to plan i = " << i);
            plan.push_back(pose);
        }
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] refined plan has " << plan.size() << "  points");
    }
    else
    {
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner ] backward global plan size:  " << plan.size());
    }
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
nav_msgs::msg::Path UndoPathGlobalPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    //ROS_WARN_NAMED("Backwards", "Backwards global planner: Generating global plan ");
    //ROS_WARN_NAMED("Backwards", "Clearing...");
    
    std::vector<geometry_msgs::msg::PoseStamped> plan;

    auto forcedGoal = lastForwardPathMsg_.poses[lastForwardPathMsg_.poses.size() - 1]; // FORCE LAST POSE
    this->createDefaultUndoPathPlan(start, forcedGoal, plan);
    //this->createPureSpiningAndStragihtLineBackwardPath(start, goal, plan);

    //ROS_INFO_STREAM(" start - " << start);
    //ROS_INFO_STREAM(" end - " << goal.pose.position);

    //ROS_INFO("3 - heading to goal orientation");
    //double goalOrientation = angles::normalize_angle(tf::getYaw(goal.pose.orientation));
    //cl_move_base_z::makePureSpinningSubPlan(prevState,goalOrientation,plan);

    //ROS_WARN_STREAM( "MAKE PLAN INVOKED, plan size:"<< plan.size());
    publishGoalMarker(forcedGoal.pose, 1.0, 0, 1.0);

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

    planPub_->publish(planMsg);
    return planMsg;

    // this was previously set to size() <= 1, but a plan with a single point is also a valid plan (the goal)
}

} // namespace undo_path_global_planner
} // namespace cl_move_base_z

PLUGINLIB_EXPORT_CLASS(cl_move_base_z::undo_path_global_planner::UndoPathGlobalPlanner, nav2_core::GlobalPlanner)