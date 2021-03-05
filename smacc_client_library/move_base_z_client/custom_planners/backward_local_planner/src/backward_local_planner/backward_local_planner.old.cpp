#include <angles/angles.h>
#include <backward_local_planner/backward_local_planner.h>
#include <move_base_z_planners_common/common.h>

#include <boost/intrusive_ptr.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::backward_local_planner::BackwardLocalPlanner, nav2_core::Controller)

using namespace std::literals::chrono_literals;

namespace cl_move_base_z
{
namespace backward_local_planner
{
/**
 ******************************************************************************************************************
 * BackwardLocalPlanner()
 ******************************************************************************************************************
 */
BackwardLocalPlanner::BackwardLocalPlanner() 
  : waitingTimeout_(0s)
//: paramServer_(ros::NodeHandle("~BackwardLocalPlanner"))
{
}

/**
 ******************************************************************************************************************
 * ~BackwardLocalPlanner()
 ******************************************************************************************************************
 */
BackwardLocalPlanner::~BackwardLocalPlanner()
{
}

void BackwardLocalPlanner::activate()
{
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"activating controller BackwardLocalPlanner");
    this->updateParameters();
    this->goalMarkerPublisher_->on_activate();
}

void BackwardLocalPlanner::deactivate()
{
    
}

void BackwardLocalPlanner::cleanup()
{
    this->backwardsPlanPath_.clear();
    this->currentCarrotPoseIndex_= 0;
}


/**
 ******************************************************************************************************************
 * BackwardLocalPlanner::configure()
 ******************************************************************************************************************
 */

template <typename T>
void tryGetOrSet(rclcpp_lifecycle::LifecycleNode::SharedPtr &node, std::string param, T& value)
{

  if(!node->get_parameter(param,value))
  {
    node->set_parameter(rclcpp::Parameter(param, value));
  }
}

template <typename T>
void declareOrSet(rclcpp_lifecycle::LifecycleNode::SharedPtr &node, std::string param, T& value)
{
  if(!node->has_parameter(param))
  {
    node->declare_parameter(param, value);
    //node->set_parameter(rclcpp::Parameter(param, value));
  }
}


void BackwardLocalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  this->costmapRos_ = costmap_ros;
  // ros::NodeHandle nh("~/BackwardLocalPlanner");
  this->nh_ = parent.lock();
  this->name_ = name;

  k_rho_ = -1.0;
  k_alpha_ = 0.5;
  k_betta_ = -1.0;  // set to zero means that orientation is not important
  carrot_angular_distance_ = 0.4;
  linear_mode_rho_error_threshold_ = 0.02;
  pureSpinningMode_ = true;
  yaw_goal_tolerance_ = 0.05;
  xy_goal_tolerance_ = 0.10;
  max_linear_x_speed_ = 1.0;
  max_angular_z_speed_ = 2.0;

  // constructor(): paramServer_(ros::NodeHandle("~BackwardLocalPlanner"))
  // f = boost::bind(&BackwardLocalPlanner::reconfigCB, this, _1, _2);
  // paramServer_.setCallback(f);

  this->currentCarrotPoseIndex_ = 0;

  declareOrSet(nh_, name_ + ".pure_spinning_straight_line_mode", pureSpinningMode_);
  
  declareOrSet(nh_, name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  declareOrSet(nh_, name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  declareOrSet(nh_, name_ + ".k_rho", k_rho_);
  declareOrSet(nh_, name_ + ".k_alpha", k_alpha_);
  declareOrSet(nh_, name_ + ".k_betta", k_betta_);
  declareOrSet(nh_, name_ + ".linear_mode_rho_error_threshold", linear_mode_rho_error_threshold_);

  declareOrSet(nh_, name_ + ".carrot_distance", carrot_distance_);
  declareOrSet(nh_, name_ + ".carrot_angular_distance", carrot_angular_distance_);
  declareOrSet(nh_, name_ + ".enable_obstacle_checking", enable_obstacle_checking_);

  declareOrSet(nh_, name_ + ".max_linear_x_speed", max_linear_x_speed_);
  declareOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);

  // we have to do this, for example for the case we are refining the final orientation.
  // se check at some point if the carrot is reached in "goal linear distance", then we go into
  // some automatic pure-spinning mode where we only update the orientation
  // This means that if we reach the carrot with precission we go into pure spinning mode but we cannot
  // leave that point (maybe this could be improved)
  if (carrot_angular_distance_ < yaw_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] carrot_angular_distance ("
                                              << carrot_angular_distance_
                                              << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_
                                              << ") setting carrot_angular_distance = " << yaw_goal_tolerance_);
    carrot_angular_distance_ = yaw_goal_tolerance_;
  }

  if (carrot_distance_ < xy_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] carrot_linear_distance ("
                                              << carrot_distance_ << ") cannot be lower than xy_goal_tolerance_ ("
                                              << yaw_goal_tolerance_
                                              << ") setting carrot_angular_distance = " << xy_goal_tolerance_);
    carrot_distance_ = xy_goal_tolerance_;
  }

  goalMarkerPublisher_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("backward_local_planner/goal_marker", 1);
  waitingTimeout_ = rclcpp::Duration(10s);
}

void BackwardLocalPlanner::updateParameters()
{
  tryGetOrSet(nh_, name_+".k_rho", k_rho_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".k_rho:"<< k_rho_);
  tryGetOrSet(nh_, name_+".k_alpha", k_alpha_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".k_alpha:"<< k_alpha_);
  tryGetOrSet(nh_, name_+".k_betta", k_betta_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".k_betta:" << k_betta_);

  tryGetOrSet(nh_, name_+".enable_obstacle_checking", enable_obstacle_checking_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".enable_obstacle_checking: " << enable_obstacle_checking_ );

  tryGetOrSet(nh_, name_+".carrot_distance", carrot_distance_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".carrot_distance:" << carrot_distance_);
  tryGetOrSet(nh_, name_+".carrot_angular_distance", carrot_angular_distance_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".carrot_angular_distance: "<< carrot_angular_distance_);
  tryGetOrSet(nh_, name_+".xy_goal_tolerance", xy_goal_tolerance_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".xy_goal_tolerance: " << xy_goal_tolerance_);
  tryGetOrSet(nh_, name_+".yaw_goal_tolerance", yaw_goal_tolerance_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".yaw_goal_tolerance: "<< yaw_goal_tolerance_);

  tryGetOrSet(nh_, name_+".pure_spinning_straight_line_mode", pureSpinningMode_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".pure_spinning_straight_line_mode: " << pureSpinningMode_);

  tryGetOrSet(nh_, name_+".linear_mode_rho_error_threshold", linear_mode_rho_error_threshold_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".linear_mode_rho_error_threshold: " << linear_mode_rho_error_threshold_);
  tryGetOrSet(nh_, name_+".max_linear_x_speed", max_linear_x_speed_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".max_linear_x_speed: "<< max_linear_x_speed_);
  tryGetOrSet(nh_, name_+".max_angular_z_speed", max_angular_z_speed_);
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".max_angular_z_speed: "<< max_angular_z_speed_);

  if (carrot_angular_distance_ < yaw_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] carrot_angular_distance ("
                                              << carrot_angular_distance_
                                              << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_
                                              << ") setting carrot_angular_distance = " << yaw_goal_tolerance_);
    carrot_angular_distance_ = yaw_goal_tolerance_;
    nh_->set_parameter(rclcpp::Parameter(name_+".carrot_angular_distance", carrot_angular_distance_));
  }
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".carrot_angular_distance: " <<carrot_angular_distance_ );

  if (carrot_distance_ < xy_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] carrot_linear_distance ("
                                              << carrot_distance_ << ") cannot be lower than xy_goal_tolerance_ ("
                                              << yaw_goal_tolerance_
                                              << ") setting carrot_angular_distance = " << xy_goal_tolerance_);
    carrot_distance_ = xy_goal_tolerance_;
    nh_->set_parameter(rclcpp::Parameter(name_+".carrot_distance", carrot_distance_));
  }
  RCLCPP_DEBUG_STREAM(nh_->get_logger(), name_+".carrot_distance:" << carrot_distance_);
}

/**
 ******************************************************************************************************************
 * auxiliar functions
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::computeCurrentEuclideanAndAngularErrorsToCarrotGoal(
    const geometry_msgs::msg::PoseStamped &tfpose, double &dist, double &angular_error)
{
    double angle = tf::getYaw(tfpose.getRotation());
            auto &carrot_pose = backwardsPlanPath_[currentCarrotPoseIndex_];
            const geometry_msgs::Point &carrot_point = carrot_pose.pose.position;

            tf2::Quaternion carrot_orientation;
            tf2::quaternionMsgToTF(carrot_pose.pose.orientation, carrot_orientation);

            geometry_msgs::msg::Pose currentPoseDebugMsg;
            tf2::poseTFToMsg(tfpose, currentPoseDebugMsg);

            // take error from the current position to the path point
            double dx = carrot_point.x - tfpose.getOrigin().x();
            double dy = carrot_point.y - tfpose.getOrigin().y();
            dist = sqrt(dx * dx + dy * dy);

            double pangle = tf::getYaw(carrot_orientation);
            angular_error = fabs(angles::shortest_angular_distance(pangle, angle));

            ROS_DEBUG_STREAM("[BackwardLocalPlanner] Compute carrot errors. (linear " << dist << ")(angular " << angular_error << ")" << std::endl
                                                                                      << "Current carrot pose: " << std::endl
}

/**
 ******************************************************************************************************************
 * updateCarrotGoal()
 ******************************************************************************************************************
 */
bool BackwardLocalPlanner::updateCarrotGoal(const geometry_msgs::msg::PoseStamped &tfpose)
{
  ROS_DEBUG("[BackwardsLocalPlanner] --- Computing carrot pose ---");
            double disterr = 0, angleerr = 0;
            // iterate the point from the current position and backward until reaching a new goal point in the path
            // this algorithm among other advantages has that skip the looping with an eager global planner
            // that recalls the same plan (the already performed part of the plan in the current pose is skipped)
            while (currentCarrotPoseIndex_ < backwardsPlanPath_.size())
            {
                computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);

                ROS_DEBUG_STREAM("[BackwardsLocalPlanner] update carrot goal: Current index: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
                ROS_DEBUG("[BackwardsLocalPlanner] update carrot goal: linear error %lf, angular error: %lf", disterr, angleerr);

                // target pose found, goal carrot tries to escape!
                if (disterr < carrot_distance_ && angleerr < carrot_angular_distance_)
                {
                    currentCarrotPoseIndex_++;
                    resetDivergenceDetection();
                    ROS_DEBUG_STREAM("[BackwardsLocalPlanner] fw " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
                }
                else
                {
                    break;
                }
            }
            //ROS_DEBUG("[BackwardsLocalPlanner] computing angular error");
            if (currentCarrotPoseIndex_ >= backwardsPlanPath_.size() && backwardsPlanPath_.size() > 0)
            {
                currentCarrotPoseIndex_ = backwardsPlanPath_.size() - 1;
                computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);
            }

            ROS_DEBUG("[BackwardsLocalPlanner] Current index carrot goal: %d", currentCarrotPoseIndex_);
            ROS_DEBUG("[BackwardsLocalPlanner] Update carrot goal: linear error  %lf, angular error: %lf", disterr, angleerr);
            bool carrotInGoalLinear = disterr < xy_goal_tolerance_;
            ROS_DEBUG("[BackwardsLocalPlanner] carrot in goal radius: %d", carrotInGoalLinear);

            ROS_DEBUG("[BackwardsLocalPlanner] --- Computing carrot pose ---");

            return carrotInGoalLinear;
}

void BackwardLocalPlanner::resetDivergenceDetection()
{
  // this function should be called always the carrot is updated
  divergenceDetectionLastCarrotLinearDistance_ = std::numeric_limits<double>::max();
  return true;

}

bool BackwardLocalPlanner::divergenceDetectionUpdate(const geometry_msgs::msg::PoseStamped &tfpose)
{
   double disterr = 0, angleerr = 0;
            computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);

            ROS_DEBUG_STREAM("[BackwardLocalPlanner] Divergence check. carrot goal distance. was: " << divergenceDetectionLastCarrotLinearDistance_ << ", now it is: " << disterr);
            if (disterr > divergenceDetectionLastCarrotLinearDistance_)
            {
                // candidate of divergence, we do not throw the divergence alarm yet
                // but we neither update the distance since it is worse than the one
                // we had previously with the same carrot.
                const double MARGIN_FACTOR = 1.2;
                if (disterr > MARGIN_FACTOR * divergenceDetectionLastCarrotLinearDistance_)
                {
                    ROS_ERROR_STREAM("[BackwardLocalPlanner] Divergence detected. The same carrot goal distance was previously: " << divergenceDetectionLastCarrotLinearDistance_ << "but now it is: " << disterr);
                    return true;
                }
                else
                {
                    // divergence candidate
                    return false;
                }
            }
            else
            {
                //update:
                divergenceDetectionLastCarrotLinearDistance_ = disterr;
                return false;
            }
}

bool BackwardLocalPlanner::checkCarrotHalfPlainConstraint(const geometry_msgs::msg::PoseStamped &tfpose)
{
  // this function is specially useful when we want to reach the goal with a lot
            // of precission. We may pass the goal and then the controller enters in some
            // unstable state. With this, we are able to detect when stop moving.

            // only apply if the carrot is in goal position and also if we are not in a pure spinning behavior v!=0

            auto &carrot_pose = backwardsPlanPath_[currentCarrotPoseIndex_];
            const geometry_msgs::Point &carrot_point = carrot_pose.pose.position;
            double yaw = tf::getYaw(carrot_pose.pose.orientation);

            // direction vector
            double vx = cos(yaw);
            double vy = sin(yaw);

            // line implicit equation
            // ax + by + c = 0
            double c = -vx * carrot_point.x - vy * carrot_point.y;
            const double C_OFFSET_METERS = 0.05; // 5 cm
            double check = vx * tfpose.getOrigin().x() + vy * tfpose.getOrigin().y() + c + C_OFFSET_METERS;

            ROS_DEBUG_STREAM("[BackwardLocalPlanner] half plane contraint:" << vx << "*" << carrot_point.x << " + " << vy << "*" << carrot_point.y << " + " << c);
            ROS_DEBUG_STREAM("[BackwardLocalPlanner] constraint evaluation: " << vx << "*" << tfpose.getOrigin().x() << " + " << vy << "*" << tfpose.getOrigin().y() << " + " << c << " = " << check);

            return check < 0;
}

bool BackwardLocalPlanner::checkGoalReached(const geometry_msgs::msg::PoseStamped &tfpose, double vetta, double gamma,
                                            double angle_error, geometry_msgs::msg::Twist &cmd_vel)
{
    auto &finalgoal = backwardsPlanPath_.back();
            double gdx = finalgoal.pose.position.x - tfpose.getOrigin().x();
            double gdy = finalgoal.pose.position.y - tfpose.getOrigin().y();
            double goaldist = sqrt(gdx * gdx + gdy * gdy);

            auto abs_angle_error = fabs(angle_error);
            // final_alpha_error =
            ROS_DEBUG_STREAM("[BackwardLocalPlanner] goal check. linear dist: " << goaldist << "(" << this->xy_goal_tolerance_ << ")"
                                                                                << ", angular dist: " << abs_angle_error << "(" << this->yaw_goal_tolerance_ << ")");

            linearGoalReached = goaldist < this->xy_goal_tolerance_;

            if (abs_angle_error < this->yaw_goal_tolerance_) // 5cm
            {
                return true;
            }

            return false;
}
/**
 ******************************************************************************************************************
 * defaultBackwardCmd()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::defaultBackwardCmd(const geometry_msgs::msg::PoseStamped &tfpose, double vetta, double gamma,
                                              double alpha_error, double betta_error,
                                              geometry_msgs::msg::Twist &cmd_vel)
{
    cmd_vel.linear.x = vetta;
    cmd_vel.angular.z = gamma;
}
/**
 ******************************************************************************************************************
 * pureSpinningCmd()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::pureSpinningCmd(const geometry_msgs::msg::PoseStamped &tfpose, double vetta, double gamma,
                                           double alpha_error, double betta_error, double rho_error,
                                           geometry_msgs::msg::Twist &cmd_vel)
{
   if (rho_error > linear_mode_rho_error_threshold_) // works in straight motion mode
            {
                vetta = k_rho_ * rho_error;
                gamma = k_alpha_ * alpha_error;
            }
            else if (fabs(betta_error) >= this->yaw_goal_tolerance_) // works in pure spinning mode
            {
                vetta = 0;
                gamma = k_betta_ * betta_error;
            }
            /*
            // THIS IS NOT TRUE, IF THE CARROT REACHES THE GOAL DOES NOT MEAN THE ROBOT IS
            // IN THE CORRECT POSITION
            else if (currentCarrotPoseIndex_ >= backwardsPlanPath_.size() - 1)
            {
                vetta = 0;
                gamma = 0;
                goalReached_ = true;
                backwardsPlanPath_.clear();

                ROS_INFO_STREAM("BACKWARD LOCAL PLANNER END: Goal Reached. Stop action [rhoerror: " << rho_error<<"]");
            }*/

            cmd_vel.linear.x = vetta;
            cmd_vel.angular.z = gamma;
}

/**
 ******************************************************************************************************************
 * computeVelocityCommands()
 ******************************************************************************************************************
 */
geometry_msgs::msg::TwistStamped BackwardLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity)
{
   ROS_DEBUG("[BackwardLocalPlanner] ------------------- LOCAL PLANNER LOOP -----------------");
            geometry_msgs::PoseStamped paux;
            tf::Stamped<tf::Pose> tfpose = optionalRobotPose(costmapRos_);

            //bool divergenceDetected = this->divergenceDetectionUpdate(tfpose);
            // it is not working in the pure spinning reel example, maybe the hyperplane check is enough
            bool divergenceDetected = false;

            bool emergency_stop = false;
            if (divergenceDetected)
            {
                ROS_ERROR("[BackwardLocalPlanner] Divergence detected. Sending emergency stop.");
                emergency_stop = true;
            }

            bool carrotInLinearGoalRange = updateCarrotGoal(tfpose);
            ROS_DEBUG_STREAM("[BackwardLocalPlanner] carrot goal created");

            if (emergency_stop)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] emergency stop, exit compute commands");
                return false;
            }

            // ------ Evaluate the current context ----
            double rho_error, betta_error, alpha_error;
            

                // getting carrot goal information
                tf::Quaternion q = tfpose.getRotation();
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] carrot goal: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
                const geometry_msgs::PoseStamped &carrotgoalpose = backwardsPlanPath_[currentCarrotPoseIndex_];
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] carrot goal pose current index: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size() << ": " << carrotgoalpose);
                const geometry_msgs::Point &carrotGoalPosition = carrotgoalpose.pose.position;

                tf::Quaternion goalQ;
                tf::quaternionMsgToTF(carrotgoalpose.pose.orientation, goalQ);
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] goal orientation: " << goalQ);

                //goal orientation (global frame)
                double betta = tf::getYaw(goalQ);
                betta = betta + betta_offset_;

                double dx = carrotGoalPosition.x - tfpose.getOrigin().x();
                double dy = carrotGoalPosition.y - tfpose.getOrigin().y();

                //distance error to the targetpoint
                rho_error = sqrt(dx * dx + dy * dy);

                //heading to goal angle
                double theta = tf::getYaw(q);
                double alpha = atan2(dy, dx);
                alpha = alpha + alpha_offset_;

                alpha_error = angles::shortest_angular_distance(alpha, theta);
                betta_error = angles::shortest_angular_distance(betta, theta);
            //------------- END CONTEXT EVAL ----------

            bool linearGoalReached;
            bool currentPoseInGoal = checkCurrentPoseInGoalRange(tfpose,betta_error, linearGoalReached);

            bool carrotInFinalGoal = carrotInLinearGoalRange && currentCarrotPoseIndex_ == backwardsPlanPath_.size() - 1;

            if(currentPoseInGoal && carrotInFinalGoal)
            {
                goalReached_ = true;
                backwardsPlanPath_.clear();
                ROS_INFO_STREAM(" [BackwardLocalPlanner] GOAL REACHED. Send stop command and skipping trajectory collision: " << cmd_vel);
                cmd_vel.linear.x =0;
                cmd_vel.angular.z = 0;
                return true;
            }

            if (carrotInLinearGoalRange && linearGoalReached) // we miss here and not any carrot ahead is outside goal, replacing carrotInLinearGoalRange to carrotInFinalLinearGoalRange
            {
                inGoalPureSpinningState_ = true;
            }


                double vetta = k_rho_ * rho_error;
                double gamma = k_alpha_ * alpha_error + k_betta_ * betta_error;
                // --------------------

                if (straightBackwardsAndPureSpinningMode_)
                {
                    this->straightBackwardsAndPureSpinCmd(tfpose, vetta, gamma, alpha_error, betta_error, rho_error, cmd_vel);
                }
                else // default curved backward-free motion mode
                {
                    // case B: goal position reached but orientation not yet reached. deactivate linear motion.
                    if (inGoalPureSpinningState_)
                    {
                        ROS_DEBUG("[BackwardLocalPlanner] we entered in a pure spinning state even in not pure-spining configuration, carrotDistanceGoalReached: %d", carrotInLinearGoalRange);
                        gamma = k_betta_ * betta_error;
                        vetta = 0;
                    }

                    //clasical control to reach a goal backwards
                    this->defaultBackwardCmd(tfpose, vetta, gamma, alpha_error, betta_error, cmd_vel);
                }

                if (cmd_vel.linear.x > max_linear_x_speed_)
                {
                    cmd_vel.linear.x = max_linear_x_speed_;
                }
                else if (cmd_vel.linear.x < -max_linear_x_speed_)
                {
                    cmd_vel.linear.x = -max_linear_x_speed_;
                }

                if (cmd_vel.angular.z > max_angular_z_speed_)
                {
                    cmd_vel.angular.z = max_angular_z_speed_;
                }
                else if (cmd_vel.angular.z < -max_angular_z_speed_)
                {
                    cmd_vel.angular.z = -max_angular_z_speed_;
                }

                publishGoalMarker(carrotGoalPosition.x, carrotGoalPosition.y, betta);

                ROS_DEBUG_STREAM("[BackwardLocalPlanner] local planner," << std::endl
                                                                         << " straightAnPureSpiningMode: " << straightBackwardsAndPureSpinningMode_ << std::endl
                                                                         << " inGoalPureSpinningState: " << inGoalPureSpinningState_ << std::endl
                                                                         << "carrotInLinearGoalRange: " << carrotInLinearGoalRange << std::endl
                                                                         << " theta: " << theta << std::endl
                                                                         << " betta: " << theta << std::endl
                                                                         << " err_x: " << dx << std::endl
                                                                         << " err_y:" << dy << std::endl
                                                                         << " rho_error:" << rho_error << std::endl
                                                                         << " alpha_error:" << alpha_error << std::endl
                                                                         << " betta_error:" << betta_error << std::endl
                                                                         << " vetta:" << vetta << std::endl
                                                                         << " gamma:" << gamma << std::endl
                                                                         << " cmd_vel.lin.x:" << cmd_vel.linear.x << std::endl
                                                                         << " cmd_vel.ang.z:" << cmd_vel.angular.z);

                if (inGoalPureSpinningState_)
                {
                    bool carrotHalfPlaneConstraintFailure = checkCarrotHalfPlainConstraint(tfpose);

                    if (carrotHalfPlaneConstraintFailure)
                    {
                        ROS_ERROR("[BackwardLocalPlanner] CarrotHalfPlaneConstraintFailure detected. Sending emergency stop and success to the planner.");
                        cmd_vel.linear.x = 0;
                    }
                }
        

            // ---------------------- TRAJECTORY PREDICTION AND COLLISION AVOIDANCE ---------------------
            //cmd_vel.linear.x=0;
            //cmd_vel.angular.z = 0;
            tf::Stamped<tf::Pose> global_pose = optionalRobotPose(costmapRos_);

            auto *costmap2d = costmapRos_->getCostmap();
            auto yaw = tf::getYaw(global_pose.getRotation());

            auto &pos = global_pose.getOrigin();

            Eigen::Vector3f currentpose(pos.x(), pos.y(), yaw);
            Eigen::Vector3f currentvel(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            std::vector<Eigen::Vector3f> trajectory;
            this->generateTrajectory(currentpose, currentvel, 0.8 /*meters*/, M_PI / 8 /*rads*/, 3.0 /*seconds*/, 0.05 /*seconds*/, trajectory);

            // check plan rejection
            bool acceptedLocalTrajectoryFreeOfObstacles = true;

            unsigned int mx, my;

            if (this->enable_obstacle_checking_)
            {
                if (backwardsPlanPath_.size() > 0)
                {
                    auto &finalgoalpose = backwardsPlanPath_.back();

                    int i = 0;
                    // ROS_INFO_STREAM("lplanner goal: " << finalgoalpose.pose.position);
                    for (auto &p : trajectory)
                    {
                        float dx = p[0] - finalgoalpose.pose.position.x;
                        float dy = p[1] - finalgoalpose.pose.position.y;

                        float dst = sqrt(dx * dx + dy * dy);
                        if (dst < xy_goal_tolerance_)
                        {
                            ROS_DEBUG("[BackwardLocalPlanner] trajectory simulation for collision checking: goal reached with no collision");
                            break;
                        }

                        costmap2d->worldToMap(p[0], p[1], mx, my);
                        unsigned int cost = costmap2d->getCost(mx, my);

                        // ROS_DEBUG("[BackwardLocalPlanner] checking cost pt %d [%lf, %lf] cell[%d,%d] = %d", i, p[0], p[1], mx, my, cost);
                        // ROS_DEBUG_STREAM("[BackwardLocalPlanner] cost: " << cost);

                        // static const unsigned char NO_INFORMATION = 255;
                        // static const unsigned char LETHAL_OBSTACLE = 254;
                        // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
                        // static const unsigned char FREE_SPACE = 0;

                        if (costmap2d->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                        {
                            acceptedLocalTrajectoryFreeOfObstacles = false;
                            ROS_WARN_STREAM("[BackwardLocalPlanner] ABORTED LOCAL PLAN BECAUSE OBSTACLE DETEDTED at point " << i << "/" << trajectory.size() << std::endl
                                                                                                                            << p[0] << ", " << p[1]);
                            break;
                        }
                        i++;
                    }
                }
                else
                {
                    ROS_WARN("[BackwardLocalPlanner] Abort local - Backwards global plan size: %ld", backwardsPlanPath_.size());
                    return false;
                }
            }

            if (acceptedLocalTrajectoryFreeOfObstacles)
            {
                waiting_ = false;
                ROS_DEBUG("[BackwardLocalPlanner] accepted local trajectory free of obstacle. Local planner continues.");
                return true;
            }
            else // that is not appceted because existence of obstacles
            {
                // emergency stop for collision: waiting a while before sending error
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;

                if (waiting_ == false)
                {
                    waiting_ = true;
                    waitingStamp_ = ros::Time::now();
                    ROS_WARN("[BackwardLocalPlanner][Not accepted local plan] starting countdown");
                }
                else
                {
                    auto waitingduration = ros::Time::now() - waitingStamp_;

                    if (waitingduration > this->waitingTimeout_)
                    {
                        ROS_WARN("[BackwardLocalPlanner][Abort local] timeout! duration %lf/%f", waitingduration.toSec(), waitingTimeout_.toSec());
                        return false;
                    }
                }

                return true;
            }
}

void BackwardLocalPlanner::setSpeedLimit(const double & speed_limit)
{
    RCLCPP_WARN_STREAM(nh_->get_logger(), "BackwardLocalPlanner::setSpeedLimit invoked. Ignored, funcionality not implemented.");
}


/**
 ******************************************************************************************************************
 * reconfigCB()
 ******************************************************************************************************************
 */
// void BackwardLocalPlanner::reconfigCB(::backward_local_planner::BackwardLocalPlannerConfig &config, uint32_t level)
// {
//     RCLCPP_INFO(nh_->get_logger(),"[BackwardLocalPlanner] reconfigure Request");
//     k_alpha_ = config.k_alpha;
//     k_betta_ = config.k_betta;
//     k_rho_ = config.k_rho;
//     enable_obstacle_checking_ = config.enable_obstacle_checking;

//     carrot_distance_ = config.carrot_distance;
//     carrot_angular_distance_ = config.carrot_angular_distance;
//     xy_goal_tolerance_ = config.xy_goal_tolerance;
//     yaw_goal_tolerance_ = config.yaw_goal_tolerance;
//     pureSpinningMode_ = config.pure_spinning_straight_line_mode;

//     if (carrot_angular_distance_ < yaw_goal_tolerance_)
//     {
//         RCLCPP_WARN_STREAM(nh_->get_logger(),"[BackwardLocalPlanner] carrot_angular_distance (" <<
//         carrot_angular_distance_ << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_ << ")
//         setting carrot_angular_distance = " << yaw_goal_tolerance_); carrot_angular_distance_ = yaw_goal_tolerance_;
//         config.carrot_angular_distance = yaw_goal_tolerance_;
//     }

//     if (carrot_distance_ < xy_goal_tolerance_)
//     {
//         RCLCPP_WARN_STREAM(nh_->get_logger(),"[BackwardLocalPlanner] carrot_linear_distance (" << carrot_distance_ <<
//         ") cannot be lower than xy_goal_tolerance_ (" << yaw_goal_tolerance_ << ") setting carrot_angular_distance =
//         " << xy_goal_tolerance_); carrot_distance_ = xy_goal_tolerance_; config.carrot_distance = xy_goal_tolerance_;
//     }
// }

/**
 ******************************************************************************************************************
 * isGoalReached()
 ******************************************************************************************************************
 */
bool BackwardLocalPlanner::isGoalReached()
{
  RCLCPP_DEBUG(nh_->get_logger(), "[BackwardLocalPlanner] isGoalReached call");
  return goalReached_;
}

bool BackwardLocalPlanner::findInitialCarrotGoal(geometry_msgs::msg::PoseStamped &tfpose)
{
  double lineardisterr, angleerr;
            bool inCarrotRange = false;

            // initial state check
            computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, lineardisterr, angleerr);

            int closestIndex = -1;
            double minpointdist = std::numeric_limits<double>::max();

            // lets set the carrot-goal in the corret place with this loop
            while (currentCarrotPoseIndex_ < backwardsPlanPath_.size() && !inCarrotRange)
            {
                computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, lineardisterr, angleerr);

                ROS_DEBUG("[BackwardLocalPlanner] Finding initial carrot goal i=%d - error to carrot, linear = %lf (%lf), angular : %lf (%lf)", currentCarrotPoseIndex_, lineardisterr, carrot_distance_, angleerr, carrot_angular_distance_);

                // current path point is inside the carrot distance range, goal carrot tries to escape!
                if (lineardisterr < carrot_distance_ && angleerr < carrot_angular_distance_)
                {
                    ROS_DEBUG("[BackwardLocalPlanner] Finding initial carrot goal i=%d - in carrot Range", currentCarrotPoseIndex_);
                    inCarrotRange = true;
                    // we are inside the goal range
                }
                else if (inCarrotRange && (lineardisterr > carrot_distance_ || angleerr > carrot_angular_distance_))
                {
                    // we were inside the carrot range but not anymore, now we are just leaving. we want to continue forward (currentCarrotPoseIndex_++)
                    // unless we go out of the carrot range

                    // but we rollback last index increment (to go back inside the carrot goal scope) and start motion with that carrot goal we found
                    currentCarrotPoseIndex_--;
                    break;
                }
                else
                {
                    ROS_DEBUG("[BackwardLocalPlanner] Finding initial carrot goal i=%d - carrot out of range, searching coincidence...", currentCarrotPoseIndex_);
                }

                currentCarrotPoseIndex_++;
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] setPlan: fw" << currentCarrotPoseIndex_);
            }

            ROS_INFO_STREAM("[BackwardLocalPlanner] setPlan: (found first carrot:" << inCarrotRange << ") initial carrot point index: " << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());

            return inCarrotRange;
}

bool BackwardLocalPlanner::resamplePrecisePlan()
{
  // this algorithm is really important to have a precise carrot (linear or angular)
            // and not being considered as a divergence from the path

            ROS_DEBUG("[BackwardLocalPlanner] resample precise");
            if(backwardsPlanPath_.size() <=1)
            {
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] resample precise skipping, size: " << backwardsPlanPath_.size());
                return false;
            }

            int counter = 0;
            double maxallowedAngularError = 0.45 * this->carrot_angular_distance_; // nyquist
            double maxallowedLinearError = 0.45 * this->carrot_distance_;          // nyquist

            for (int i = 0; i < backwardsPlanPath_.size() - 1; i++)
            {
                ROS_DEBUG_STREAM("[BackwardLocalPlanner] resample precise, check: " << i);
                auto &currpose = backwardsPlanPath_[i];
                auto &nextpose = backwardsPlanPath_[i + 1];

                tf::Quaternion qCurrent, qNext;
                tf::quaternionMsgToTF(currpose.pose.orientation, qCurrent);
                tf::quaternionMsgToTF(nextpose.pose.orientation, qNext);

                double dx = nextpose.pose.position.x - currpose.pose.position.x;
                double dy = nextpose.pose.position.y - currpose.pose.position.y;
                double dist = sqrt(dx * dx + dy * dy);

                bool resample = false;
                if (dist > maxallowedLinearError)
                {
                    ROS_DEBUG_STREAM("[BackwardLocalPlanner] resampling point, linear distance:" << dist << "(" << maxallowedLinearError << ")" << i);
                    resample = true;
                }
                else
                {
                    double currentAngle = tf::getYaw(qCurrent);
                    double nextAngle = tf::getYaw(qNext);

                    double angularError = fabs(angles::shortest_angular_distance(currentAngle, nextAngle));
                    if (angularError > maxallowedAngularError)
                    {
                        resample = true;
                        ROS_DEBUG_STREAM("[BackwardLocalPlanner] resampling point, angular distance:" << angularError << "(" << maxallowedAngularError << ")" << i);
                    }
                }

                if (resample)
                {
                    geometry_msgs::PoseStamped pintermediate;
                    auto duration = nextpose.header.stamp - currpose.header.stamp;
                    pintermediate.header.frame_id = currpose.header.frame_id;
                    pintermediate.header.stamp = currpose.header.stamp + duration *0.5;

                    pintermediate.pose.position.x = 0.5 * (currpose.pose.position.x + nextpose.pose.position.x);
                    pintermediate.pose.position.y = 0.5 * (currpose.pose.position.y + nextpose.pose.position.y);
                    pintermediate.pose.position.z = 0.5 * (currpose.pose.position.z + nextpose.pose.position.z);
                    tf::Quaternion intermediateQuat = tf::slerp(qCurrent, qNext, 0.5);
                    tf::quaternionTFToMsg(intermediateQuat, pintermediate.pose.orientation);

                    this->backwardsPlanPath_.insert(this->backwardsPlanPath_.begin() + i + 1, pintermediate);

                    // retry this point
                    i--;
                    counter++;
                }
            }

            ROS_DEBUG_STREAM("[BackwardLocalPlanner] End resampling. resampled:" << counter << " new inserted poses during precise resmapling.");
            return true;
}

/**
 ******************************************************************************************************************
 * setPlan()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::setPlan(const nav_msgs::msg::Path &path)
{
    ROS_INFO_STREAM("[BackwardLocalPlanner] setPlan: new global plan received (" << plan.size() << ")");
            initialPureSpinningStage_ = true;
            goalReached_ = false;
            backwardsPlanPath_ = plan;
            inGoalPureSpinningState_ =false;

            // find again the new carrot goal, from the destiny direction
            tf::Stamped<tf::Pose> tfpose = optionalRobotPose(costmapRos_);

            geometry_msgs::PoseStamped posestamped;
            tf::poseStampedTFToMsg(tfpose,posestamped);
            backwardsPlanPath_.insert(backwardsPlanPath_.begin(), posestamped);

            this->resamplePrecisePlan();

            currentCarrotPoseIndex_ = 0;
            this->resetDivergenceDetection();

            if (plan.size() == 0)
            {
                ROS_WARN("[BackwardLocalPlanner] received plan without any pose");
                return true;
            }

            bool foundInitialCarrotGoal = this->findInitialCarrotGoal(tfpose);
            if (!foundInitialCarrotGoal)
            {
                ROS_ERROR("[BackwardLocalPlanner] new plan rejected. The initial point in the global path is too much far away from the current state (according to carrot_distance parameter)");
                return false; // in this case, the new plan broke the current execution
            }
            else
            {
                this->divergenceDetectionUpdate(tfpose);
                // SANDARD AND PREFERED CASE ON NEW PLAN
                return true;
            }
}

void BackwardLocalPlanner::generateTrajectory(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, float maxdist,
                                              float maxanglediff, float maxtime, float dt,
                                              std::vector<Eigen::Vector3f> &outtraj)
{
   //simulate the trajectory and check for collisions, updating costs along the way
            bool end = false;
            float time = 0;
            Eigen::Vector3f currentpos = pos;
            int i = 0;
            while (!end)
            {
                //add the point to the trajectory so we can draw it later if we want
                //traj.addPoint(pos[0], pos[1], pos[2]);

                // if (continued_acceleration_) {
                //   //calculate velocities
                //   loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
                //   //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
                // }

                auto loop_vel = vel;
                //update the position of the robot using the velocities passed in
                auto newpos = computeNewPositions(currentpos, loop_vel, dt);

                auto dx = newpos[0] - currentpos[0];
                auto dy = newpos[1] - currentpos[1];
                float dist, angledist;

                //ROS_INFO("traj point %d", i);
                dist = sqrt(dx * dx + dy * dy);
                if (dist > maxdist)
                {
                    end = true;
                    //ROS_INFO("dist break: %f", dist);
                }
                else
                {
                    // ouble from, double to
                    angledist = angles::shortest_angular_distance(currentpos[2], newpos[2]);
                    if (angledist > maxanglediff)
                    {
                        end = true;
                        //ROS_INFO("angle dist break: %f", angledist);
                    }
                    else
                    {
                        outtraj.push_back(newpos);

                        time += dt;
                        if (time > maxtime)
                        {
                            end = true;
                            //ROS_INFO("time break: %f", time);
                        }

                        //ROS_INFO("dist: %f, angledist: %f, time: %f", dist, angledist, time);
                    }
                }

                currentpos = newpos;
                i++;
            } // end for simulation steps
}

Eigen::Vector3f BackwardLocalPlanner::computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel,
                                                          double dt)
{
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 ******************************************************************************************************************
 * publishGoalMarker()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::publishGoalMarker(double x, double y, double phi)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->costmapRos_->getGlobalFrameID();
  marker.header.stamp = nh_->now();

  marker.ns = "my_namespace2";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1;

  marker.scale.x = 0.05;
  marker.scale.y = 0.15;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;

  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;

  geometry_msgs::msg::Point start, end;
  start.x = x;
  start.y = y;

  end.x = x + 0.5 * cos(phi);
  end.y = y + 0.5 * sin(phi);

  marker.points.push_back(start);
  marker.points.push_back(end);

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(marker);

  goalMarkerPublisher_->publish(ma);
}
}  // namespace backward_local_planner
}  // namespace cl_move_base_z