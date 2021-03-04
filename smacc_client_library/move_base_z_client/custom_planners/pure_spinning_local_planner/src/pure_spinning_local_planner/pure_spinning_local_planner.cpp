
#include <angles/angles.h>
#include <pure_spinning_local_planner/pure_spinning_local_planner.h>

#include <pluginlib/class_list_macros.hpp>

namespace cl_move_base_z
{
namespace pure_spinning_local_planner
{
PureSpinningLocalPlanner::PureSpinningLocalPlanner()
{
}

PureSpinningLocalPlanner::~PureSpinningLocalPlanner()
{
}

void PureSpinningLocalPlanner::activate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "activating controller PureSpinningLocalPlanner");
  this->updateParameters();
}

void PureSpinningLocalPlanner::deactivate()
{
}

void PureSpinningLocalPlanner::cleanup()
{
  this->plan_.clear();
  this->currentPoseIndex_ = 0;
}

void PureSpinningLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &node, std::string name,
                                         const std::shared_ptr<tf2_ros::Buffer> & /*tf*/,
                                         const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros)
{
  costmapRos_ = costmap_ros;
  name_ = name;
  nh_ = node.lock();
  k_betta_ = 10.0;
  yaw_goal_tolerance_ = 0.08;
  intermediate_goal_yaw_tolerance_ = 0.12;
  max_angular_z_speed_ = 1.0;

  nh_->declare_parameter("k_betta", k_betta_);
  nh_->declare_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  nh_->declare_parameter("intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_);
  nh_->declare_parameter("max_angular_z_speed", max_angular_z_speed_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] pure spinning planner already created");
}

void PureSpinningLocalPlanner::updateParameters()
{
  nh_->get_parameter("k_betta", k_betta_);
  nh_->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  nh_->get_parameter("intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_);
  nh_->get_parameter("max_angular_z_speed", max_angular_z_speed_);
}

geometry_msgs::msg::TwistStamped PureSpinningLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist & /*velocity*/)
{
  this->updateParameters();
  geometry_msgs::msg::TwistStamped cmd_vel;

  goalReached_ = false;
  // ROS_DEBUG("LOCAL PLANNER LOOP");

  geometry_msgs::msg::PoseStamped currentPose = pose;

  tf2::Quaternion q;
  tf2::fromMsg(currentPose.pose.orientation, q);
  auto currentYaw = tf2::getYaw(currentPose.pose.orientation);
  double angular_error;
  double targetYaw;

  while (currentPoseIndex_ < (int)plan_.size())
  {
    auto &goal = plan_[currentPoseIndex_];
    targetYaw = tf2::getYaw(goal.pose.orientation);

    // double angular_error = angles::shortest_angular_distance( currentYaw , targetYaw) ;
    angular_error = targetYaw - currentYaw;

    // all the points must be reached using the control rule, but the last one
    // have an special condition
    if ((currentPoseIndex_ < (int)plan_.size() - 1 && fabs(angular_error) < this->intermediate_goal_yaw_tolerance_) ||
        (currentPoseIndex_ == (int)plan_.size() - 1 && fabs(angular_error) < this->intermediate_goal_yaw_tolerance_))
    {
      currentPoseIndex_++;
    }
    else
    {
      break;
    }
  }

  auto omega = angular_error * k_betta_;
  cmd_vel.twist.angular.z = std::min(std::max(omega, -fabs(max_angular_z_speed_)), fabs(max_angular_z_speed_));

  RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] angular error: " << angular_error << "("
                                                                                      << yaw_goal_tolerance_ << ")");
  RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                      "[PureSpinningLocalPlanner] command angular speed: " << cmd_vel.twist.angular.z);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(),
                      "[PureSpinningLocalPlanner] completion " << currentPoseIndex_ << "/" << plan_.size());

  if (currentPoseIndex_ >= (int)plan_.size() - 1 && fabs(angular_error) < yaw_goal_tolerance_)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] GOAL REACHED ");
    goalReached_ = true;
  }

  return cmd_vel;
}

bool PureSpinningLocalPlanner::isGoalReached()
{
  return goalReached_;
}

void PureSpinningLocalPlanner::setPlan(const nav_msgs::msg::Path &path)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "activating controller PureSpinningLocalPlanner");
  plan_ = path.poses;
  goalReached_ = false;
  currentPoseIndex_ = 0;
}

void publishGoalMarker(double x, double y, double phi)
{
}
}  // namespace cl_move_base_z
}  // namespace cl_move_base_z

PLUGINLIB_EXPORT_CLASS(cl_move_base_z::pure_spinning_local_planner::PureSpinningLocalPlanner, nav2_core::Controller)