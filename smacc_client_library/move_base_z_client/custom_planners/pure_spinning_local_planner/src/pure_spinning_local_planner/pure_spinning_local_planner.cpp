
#include <angles/angles.h>
#include <pure_spinning_local_planner/pure_spinning_local_planner.h>

#include <pluginlib/class_list_macros.hpp>

namespace cl_move_base_z
{
namespace pure_spinning_local_planner
{
template <typename T>
void tryGetOrSet(rclcpp_lifecycle::LifecycleNode::SharedPtr &node, std::string param, T &value)
{
  if (!node->get_parameter(param, value))
  {
    node->set_parameter(rclcpp::Parameter(param, value));
  }
}

template <typename T>
void declareOrSet(rclcpp_lifecycle::LifecycleNode::SharedPtr &node, std::string param, T &value)
{
  if (!node->has_parameter(param))
  {
    node->declare_parameter(param, value);
    // node->set_parameter(rclcpp::Parameter(param, value));
  }
}

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
  yaw_goal_tolerance_ = -1.0;
}

void PureSpinningLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &node, std::string name,
                                         const std::shared_ptr<tf2_ros::Buffer> & /*tf*/,
                                         const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros)
{
  costmapRos_ = costmap_ros;
  name_ = name;
  nh_ = node.lock();
  k_betta_ = 1.0;
  intermediate_goal_yaw_tolerance_ = 0.12;
  max_angular_z_speed_ = 1.0;
  yaw_goal_tolerance_ = -1.0;

  declareOrSet(nh_, name_ + ".k_betta", k_betta_);
  declareOrSet(nh_, name_ + ".intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_);
  declareOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);
  
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] pure spinning planner already created");
}

void PureSpinningLocalPlanner::updateParameters()
{
  tryGetOrSet(nh_, name_ + ".k_betta", k_betta_);
  tryGetOrSet(nh_, name_ + ".intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_);
  tryGetOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);
}

geometry_msgs::msg::TwistStamped PureSpinningLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
  this->updateParameters();
  if (yaw_goal_tolerance_ == -1)
  {
    geometry_msgs::msg::Pose posetol;
    geometry_msgs::msg::Twist twistol;
    if (goal_checker->getTolerances(posetol, twistol))
    {
      yaw_goal_tolerance_ = tf2::getYaw(posetol.orientation) * 0.35; // WORKAROUND GOAL CHECKER DIFFERENCE NAV CONTROLLER
      RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] yaw_goal_tolerance_: " << yaw_goal_tolerance_);
    }
    else
    {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] could not get tolerances from goal checker");
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;

  goalReached_ = false;
  // RCLCPP_DEBUG(nh_->get_logger(), "LOCAL PLANNER LOOP");

  geometry_msgs::msg::PoseStamped currentPose = pose;

  tf2::Quaternion q;
  tf2::fromMsg(currentPose.pose.orientation, q);
  auto currentYaw = tf2::getYaw(currentPose.pose.orientation);
  double angular_error=0;
  double targetYaw;

  do
  {
    auto &goal = plan_[currentPoseIndex_];
    targetYaw = tf2::getYaw(goal.pose.orientation);

    angular_error = angles::shortest_angular_distance(currentYaw, targetYaw);

    // if it is in the following way, sometimes the direction is incorrect
    //angular_error = targetYaw - currentYaw;

    // all the points must be reached using the control rule, but the last one
    // have an special condition
    if ((currentPoseIndex_ < (int)plan_.size() - 1 && fabs(angular_error) < this->intermediate_goal_yaw_tolerance_))
    {
      currentPoseIndex_++;
    }
    else
    {
      break;
    }
  }while (currentPoseIndex_ < (int)plan_.size());

  auto omega = angular_error * k_betta_;
  cmd_vel.twist.angular.z = std::min(std::max(omega, -fabs(max_angular_z_speed_)), fabs(max_angular_z_speed_));

  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "[PureSpinningLocalPlanner] pose index: " << currentPoseIndex_ << "/" << plan_.size());
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] k_betta_: " << k_betta_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] angular error: " << angular_error << "("
                                                                                     << yaw_goal_tolerance_ << ")");
  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "[PureSpinningLocalPlanner] command angular speed: " << cmd_vel.twist.angular.z);
  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "[PureSpinningLocalPlanner] completion " << currentPoseIndex_ << "/" << plan_.size());

  if (currentPoseIndex_ >= (int)plan_.size() - 1 && fabs(angular_error) < yaw_goal_tolerance_)
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] GOAL REACHED. Sending stop command.");
    goalReached_ = true;
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;
  }

  return cmd_vel;
}

void PureSpinningLocalPlanner::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  RCLCPP_WARN_STREAM(nh_->get_logger(), "PureSpinningLocalPlanner::setSpeedLimit invoked. Ignored, funcionality not "
                                        "implemented.");
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

void publishGoalMarker(double /*x*/, double /*y*/, double /*phi*/)
{
}
}  // namespace pure_spinning_local_planner
}  // namespace cl_move_base_z

PLUGINLIB_EXPORT_CLASS(cl_move_base_z::pure_spinning_local_planner::PureSpinningLocalPlanner, nav2_core::Controller)