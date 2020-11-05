
#include <move_base_z_client_plugin/client_behaviors/cb_absolute_rotate.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/common.h>

namespace cl_move_base_z
{
CbAbsoluteRotate::CbAbsoluteRotate()
{
}

CbAbsoluteRotate::CbAbsoluteRotate(float absoluteGoalAngleDegree, float yaw_goal_tolerance)
{
    this->absoluteGoalAngleDegree = absoluteGoalAngleDegree;

    if (yaw_goal_tolerance >= 0)
    {
        this->yawGoalTolerance = yaw_goal_tolerance;
    }
}

void CbAbsoluteRotate::updateTemporalBehaviorParameters(bool undo)
{
    // dynamic_reconfigure::ReconfigureRequest srv_req;
    // dynamic_reconfigure::ReconfigureResponse srv_resp;
    // dynamic_reconfigure::Config conf;

    //ros::NodeHandle nh;

    std::string nodename = "/move_base";
    std::string localPlannerName;
    //dynamic_reconfigure::DoubleParameter yaw_goal_tolerance;
    rclcpp::Parameter yaw_goal_tolerance("yaw_goal_tolerance");
    std::vector<rclcpp::Parameter> parameters;

    //dynamic_reconfigure::DoubleParameter max_vel_theta;
    //dynamic_reconfigure::DoubleParameter min_vel_theta;
    rclcpp::Parameter max_vel_theta, min_vel_theta;

    bool isRosBasePlanner = !spinningPlanner || *spinningPlanner == SpiningPlanner::Default;
    bool isPureSpinningPlanner = spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning;

    if(isPureSpinningPlanner)
    {
        localPlannerName = "PureSpinningLocalPlanner";
        max_vel_theta = rclcpp::Parameter("max_angular_z_speed");
        min_vel_theta = rclcpp::Parameter("min_vel_theta");
    }
    else if(isRosBasePlanner)
    {
        localPlannerName = "TrajectoryPlannerROS";
        max_vel_theta = rclcpp::Parameter("max_vel_theta");
        min_vel_theta = rclcpp::Parameter("min_vel_theta");
    }
    
    
    if(!undo)
    {
       if(yawGoalTolerance)
       {
            // save old yaw tolerance
            getCurrentState()->getParam(nodename + "/"  + localPlannerName+"/yaw_goal_tolerance", oldYawTolerance);
            yaw_goal_tolerance = rclcpp::Parameter("yaw_goal_tolerance",*yawGoalTolerance);
            parameters.push_back(yaw_goal_tolerance);
            RCLCPP_INFO(getNode()->get_logger(),"[CbAbsoluteRotate] updating yaw tolerance local planner to: %lf, from previous value: %lf ", *yawGoalTolerance, this->oldYawTolerance);
       }

       if(maxVelTheta)
       {
           if(isRosBasePlanner)
           {
                // save old yaw tolerance
                //nh.getParam(nodename + "/"  + localPlannerName+"/min_vel_theta", oldMinVelTheta);
                getCurrentState()->getParam(nodename + "/"  + localPlannerName+"/max_vel_theta", oldMaxVelTheta);
           }
            max_vel_theta = rclcpp::Parameter("max_vel_theta", *maxVelTheta);
            min_vel_theta = rclcpp::Parameter("min_vel_theta",  -*maxVelTheta);
            parameters.push_back(max_vel_theta);
            parameters.push_back(min_vel_theta);
            RCLCPP_INFO(getNode()->get_logger(),"[CbAbsoluteRotate] updating max vel theta local planner to: %lf, from previous value: %lf ", *maxVelTheta, this->oldMaxVelTheta);
            RCLCPP_INFO(getNode()->get_logger(),"[CbAbsoluteRotate] updating min vel theta local planner to: %lf, from previous value: %lf ", -*maxVelTheta, this->oldMinVelTheta);
       }
    }
    else
    {
        if(yawGoalTolerance)
        {
            yaw_goal_tolerance = rclcpp::Parameter("yaw_goal_tolerance",oldYawTolerance);
            RCLCPP_INFO(getNode()->get_logger(),"[CbAbsoluteRotate] restoring yaw tolerance local planner from: %lf, to previous value: %lf ", *yawGoalTolerance, this->oldYawTolerance);
        }

        if(maxVelTheta)
        {
            if(isRosBasePlanner)
            {
                max_vel_theta = rclcpp::Parameter("max_vel_theta", oldMaxVelTheta);
                min_vel_theta = rclcpp::Parameter("min_vel_theta",  oldMinVelTheta);
            }

            parameters.push_back(max_vel_theta);
            parameters.push_back(min_vel_theta);
            RCLCPP_INFO(getNode()->get_logger(),"[CbAbsoluteRotate] restoring max vel theta local planner from: %lf, to previous value: %lf ", *maxVelTheta, this->oldMaxVelTheta);
            RCLCPP_INFO(getNode()->get_logger(),"[CbAbsoluteRotate] restoring min vel theta local planner from: %lf, to previous value: %lf ", -(*maxVelTheta), this->oldMinVelTheta);
        }
    }

    // srv_req.config = conf;
    // bool res;
    // do
    // {
        std::string servername = nodename + "/"  + localPlannerName+"/set_parameters";
        
        //res = ros::service::call( servername, srv_req, srv_resp);
        auto res= getNode()->set_parameters(parameters);
        

        RCLCPP_INFO_STREAM(getNode()->get_logger(),"[CbAbsoluteRotate] dynamic configure call ["<< servername <<"]: ");
        rclcpp::spin_some(getNode());

        //if(!res)
        for(auto& r: res)
        {
            if(!r.successful)
            {
                //rclcpp::sleep_for(std::chrono::milliseconds(100));
                RCLCPP_WARN_STREAM(getNode()->get_logger(),"[CbAbsoluteRotate] Failed, retrtying call: " << r.reason);
            }
        }
    // }while(!res);
}

void CbAbsoluteRotate::onExit()
{
    if(spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning)
    {

    }
    else
    {    
    }

    this->updateTemporalBehaviorParameters(true);
}

void CbAbsoluteRotate::onEntry()
{
    double goal_angle;
    
    listener = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
    if (!this->absoluteGoalAngleDegree)
    {
        goal_angle =45.0;
        this->getCurrentState()->param("goal_angle", goal_angle);
    }
    else
    {
        goal_angle = *this->absoluteGoalAngleDegree;
    
    }
    RCLCPP_INFO_STREAM(getNode()->get_logger(),"[CbAbsoluteRotate] Absolute yaw Angle:" << goal_angle);

    auto plannerSwitcher = this->moveBaseClient_->getComponent<PlannerSwitcher>();
    //this should work better with a coroutine and await
    //this->plannerSwitcher_->setForwardPlanner();
    
    if(spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning)
        plannerSwitcher->setPureSpinningPlanner();
    else
        plannerSwitcher->setDefaultPlanners();
    
    updateTemporalBehaviorParameters(false);

    auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
    auto referenceFrame = p->getReferenceFrame();
    auto currentPoseMsg = p->toPoseMsg();

    ClMoveBaseZ::Goal goal;
    goal.pose.header.frame_id = referenceFrame;
    goal.pose.header.stamp = getNode()->now();

    auto currentAngle = tf2::getYaw(currentPoseMsg.orientation);
    auto targetAngle = goal_angle * M_PI / 180.0;
    goal.pose.pose.position = currentPoseMsg.position;
    tf2::Quaternion q;
    q.setEuler(targetAngle, 0, 0);
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto odomTracker_ = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
    if (odomTracker_!=nullptr)
    {
        odomTracker_->pushPath();
        odomTracker_->setStartPoint(p->toPoseStampedMsg());
        odomTracker_->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);
    }

    RCLCPP_INFO_STREAM(getNode()->get_logger(),"[CbAbsoluteRotate] current pose: " << currentPoseMsg);
    RCLCPP_INFO_STREAM(getNode()->get_logger(),"[CbAbsoluteRotate] goal pose: " << goal.pose.pose);
    moveBaseClient_->sendGoal(goal);
}
} // namespace cl_move_base_z
