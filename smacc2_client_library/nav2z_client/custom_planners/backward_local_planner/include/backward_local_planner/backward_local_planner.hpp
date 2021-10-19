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
//#include <dynamic_reconfigure/server.h>
//#include <backward_local_planner/BackwardLocalPlannerConfig.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <eigen3/Eigen/Eigen>

typedef double meter;
typedef double rad;

namespace cl_nav2z
{
namespace backward_local_planner
{
class BackwardLocalPlanner : public nav2_core::Controller
{
public:
  BackwardLocalPlanner();

  virtual ~BackwardLocalPlanner();

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

  // returns true if found
  bool findInitialCarrotGoal(geometry_msgs::msg::PoseStamped & pose);

  // returns true for a pure spining motion request
  bool updateCarrotGoal(const geometry_msgs::msg::PoseStamped & pose);

  bool resamplePrecisePlan();

  void straightBackwardsAndPureSpinCmd(
    const geometry_msgs::msg::PoseStamped & pose, double & vetta, double & gamma,
    double alpha_error, double betta_error, double rho_error);

  void clearMarkers();
  void publishGoalMarker(double x, double y, double phi);

  void computeCurrentEuclideanAndAngularErrorsToCarrotGoal(
    const geometry_msgs::msg::PoseStamped & pose, double & dist, double & angular_error);

  bool checkGoalReached(
    const geometry_msgs::msg::PoseStamped & pose, double vetta, double gamma, double alpha_error,
    geometry_msgs::msg::Twist & cmd_vel);

  bool checkCurrentPoseInGoalRange(
    const geometry_msgs::msg::PoseStamped & tfpose, const geometry_msgs::msg::Twist & currentTwist,
    double angle_error, bool & linearGoalReached, nav2_core::GoalChecker * goalChecker);

  bool resetDivergenceDetection();

  bool divergenceDetectionUpdate(const geometry_msgs::msg::PoseStamped & pose);

  bool checkCarrotHalfPlainConstraint(const geometry_msgs::msg::PoseStamped & pose);

  // void reconfigCB(::backward_local_planner::BackwardLocalPlannerConfig &config, uint32_t level);
  // dynamic_reconfigure::Server<::backward_local_planner::BackwardLocalPlannerConfig> paramServer_;
  // dynamic_reconfigure::Server<::backward_local_planner::BackwardLocalPlannerConfig>::CallbackType f;

  rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;
  std::string name_;

  std::vector<geometry_msgs::msg::PoseStamped> backwardsPlanPath_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    goalMarkerPublisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> planPub_;

  // --- control parameters ---
  double k_rho_;
  double k_alpha_;
  double k_betta_;
  double pure_spinning_allowed_betta_error_;
  double linear_mode_rho_error_threshold_;

  const double alpha_offset_ = M_PI;
  const double betta_offset_ = 0;

  double max_linear_x_speed_;   // meters/sec
  double max_angular_z_speed_;  // rads/sec

  double yaw_goal_tolerance_;  // radians
  double xy_goal_tolerance_;   // meters

  meter carrot_distance_;
  rad carrot_angular_distance_;
  meter divergenceDetectionLastCarrotLinearDistance_;

  double transform_tolerance_;

  rclcpp::Duration waitingTimeout_;
  rclcpp::Time waitingStamp_;

  // --- controller state ---
  bool goalReached_;
  bool initialPureSpinningStage_;
  bool straightBackwardsAndPureSpinningMode_ = false;
  bool enable_obstacle_checking_ = true;
  bool inGoalPureSpinningState_ = false;
  bool waiting_;

  // references the current point inside the backwardsPlanPath were the robot is located
  int currentCarrotPoseIndex_;

  void generateTrajectory(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & vel, float maxdist, float maxangle,
    float maxtime, float dt, std::vector<Eigen::Vector3f> & outtraj);

  Eigen::Vector3f computeNewPositions(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & vel, double dt);
};
}  // namespace backward_local_planner
}  // namespace cl_nav2z
