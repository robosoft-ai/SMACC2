// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <eigen3/Eigen/Eigen>

//#include <dynamic_reconfigure/server.h>
#include <tf2/transform_datatypes.h>
#include <nav2z_planners_common/common.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <nav2_core/controller.hpp>

typedef double meter;
typedef double rad;

namespace cl_nav2z
{
namespace forward_local_planner
{
class ForwardLocalPlanner : public nav2_core::Controller
{
public:
  ForwardLocalPlanner();

  virtual ~ForwardLocalPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief nav2_core computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @return The best command for the robot to drive
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /*deprecated in navigation2*/
  bool isGoalReached();

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void updateParameters();
  nav2_util::LifecycleNode::SharedPtr nh_;

  void publishGoalMarker(double x, double y, double phi);
  void cleanMarkers();

  void generateTrajectory(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & vel, float maxdist, float maxangle,
    float maxtime, float dt, std::vector<Eigen::Vector3f> & outtraj);
  Eigen::Vector3f computeNewPositions(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & vel, double dt);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos_;
  std::string name_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    goalMarkerPublisher_;

  double k_rho_;
  double k_alpha_;
  double k_betta_;
  bool goalReached_;

  const double alpha_offset_ = 0;
  const double betta_offset_ = 0;

  meter carrot_distance_;
  rad carrot_angular_distance_;

  double yaw_goal_tolerance_;  // radians
  double xy_goal_tolerance_;   // meters

  double max_angular_z_speed_;
  double max_linear_x_speed_;
  double transform_tolerance_;

  // references the current point inside the backwardsPlanPath were the robot is located
  int currentPoseIndex_;

  std::vector<geometry_msgs::msg::PoseStamped> plan_;

  bool waiting_;
  rclcpp::Duration waitingTimeout_;
  rclcpp::Time waitingStamp_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};
}  // namespace forward_local_planner
}  // namespace cl_nav2z
