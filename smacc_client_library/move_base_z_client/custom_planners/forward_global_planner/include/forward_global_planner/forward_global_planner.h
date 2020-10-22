/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <nav2_core/global_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

//#include <nav_msgs/GetPlan.h>
//#include <ros/ros.h>

namespace cl_move_base_z
{
namespace forward_global_planner
{
class ForwardGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<ForwardGlobalPlanner>;

  ForwardGlobalPlanner();

  /**
   * @brief Virtual destructor
   */
  virtual ~ForwardGlobalPlanner();

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup();

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() ;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() ;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

private:
    //rclcpp::Node::SharedPtr nh_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planPub_;

    /// stored but almost not used
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    
    double skip_straight_motion_distance_; //meters

    double puresSpinningRadStep_; // rads

    std::string name_;
};
} // namespace forward_global_planner
} // namespace cl_move_base_z