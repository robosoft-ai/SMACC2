/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
//#include <dynamic_reconfigure/server.h>
//#include <backward_local_planner/BackwardLocalPlannerConfig.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

typedef double meter;
typedef double rad;

namespace cl_move_base_z
{
namespace backward_local_planner
{
class BackwardLocalPlanner : public nav2_core::Controller
{
public:
  BackwardLocalPlanner();

  virtual ~BackwardLocalPlanner();

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
                 std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) override;

  void activate() override;

  void deactivate() override;

  void cleanup() override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path &path) override;

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
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                           const geometry_msgs::msg::Twist &velocity) override;

  /*deprecated in navigation2*/
  bool isGoalReached();

private:
  void updateParameters();

  // returns true if found
  bool findInitialCarrotGoal(geometry_msgs::msg::PoseStamped &pose);

  // returns true for a pure spining motion request
  bool updateCarrotGoal(const geometry_msgs::msg::PoseStamped &pose);

  bool resamplePrecisePlan();

  void pureSpinningCmd(const geometry_msgs::msg::PoseStamped &pose, double vetta, double gamma, double alpha_error,
                       double betta_error, double rho_error, geometry_msgs::msg::Twist &cmd_vel);
  void defaultBackwardCmd(const geometry_msgs::msg::PoseStamped &pose, double vetta, double gamma, double alpha_error,
                          double betta_error, geometry_msgs::msg::Twist &cmd_vel);

  void publishGoalMarker(double x, double y, double phi);
  void computeCurrentEuclideanAndAngularErrorsToCarrotGoal(const geometry_msgs::msg::PoseStamped &pose, double &dist,
                                                           double &angular_error);
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped &pose, double vetta, double gamma, double alpha_error,
                        geometry_msgs::msg::Twist &cmd_vel);

  void resetDivergenceDetection();
  bool divergenceDetectionUpdate(const geometry_msgs::msg::PoseStamped &pose);
  bool checkCarrotHalfPlainConstraint(const geometry_msgs::msg::PoseStamped &pose);

  // void reconfigCB(::backward_local_planner::BackwardLocalPlannerConfig &config, uint32_t level);
  // dynamic_reconfigure::Server<::backward_local_planner::BackwardLocalPlannerConfig> paramServer_;
  // dynamic_reconfigure::Server<::backward_local_planner::BackwardLocalPlannerConfig>::CallbackType f;

  rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;
  std::string name_;

  std::vector<geometry_msgs::msg::PoseStamped> backwardsPlanPath_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> goalMarkerPublisher_;

  double k_rho_;
  double k_alpha_;
  double k_betta_;
  double pure_spinning_allowed_betta_error_;
  double linear_mode_rho_error_threshold_;

  bool goalReached_;
  bool initialPureSpinningStage_;
  bool pureSpinningMode_ = false;
  bool enable_obstacle_checking_ = true;

  const double alpha_offset_ = M_PI;
  const double betta_offset_ = 0;

  double yaw_goal_tolerance_;  // radians
  double xy_goal_tolerance_;   // meters

  meter carrot_distance_;
  rad carrot_angular_distance_;
  meter divergenceDetectionLastCarrotLinearDistance_;

  double max_linear_x_speed_;   // meters/sec
  double max_angular_z_speed_;  // rads/sec

  // references the current point inside the backwardsPlanPath were the robot is located
  int currentCarrotPoseIndex_;

  void generateTrajectory(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, float maxdist, float maxangle,
                          float maxtime, float dt, std::vector<Eigen::Vector3f> &outtraj);

  Eigen::Vector3f computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, double dt);

  bool waiting_;
  rclcpp::Duration waitingTimeout_;
  rclcpp::Time waitingStamp_;
};
}  // namespace backward_local_planner
}  // namespace cl_move_base_z
