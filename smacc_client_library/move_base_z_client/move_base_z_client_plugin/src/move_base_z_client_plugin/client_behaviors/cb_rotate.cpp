#include <move_base_z_client_plugin/client_behaviors/cb_rotate.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <move_base_z_client_plugin/common.h>

namespace cl_move_base_z
{
CbRotate::CbRotate()
{
}

CbRotate::CbRotate(float rotate_degree)
{
    rotateDegree = rotate_degree;
}

void CbRotate::onEntry()
{
    double angle_increment_degree = 45.0;

    if (!rotateDegree)
    {
        this->getCurrentState()->getParam("angle_increment_degree", angle_increment_degree);
    }
    else
    {
        angle_increment_degree = *rotateDegree;
    }

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    //this should work better with a coroutine and await
    //this->plannerSwitcher_->setForwardPlanner();
    plannerSwitcher->setDefaultPlanners();

    auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
    auto referenceFrame = p->getReferenceFrame();
    auto currentPoseMsg = p->toPoseMsg();

    tf2::Transform currentPose;
    tf2::fromMsg(currentPoseMsg, currentPose);

    auto odomTracker = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
    ClMoveBaseZ::Goal goal;
    goal.pose.header.frame_id = referenceFrame;
    goal.pose.header.stamp = getNode()->now();

    auto currentAngle = tf2::getYaw(currentPoseMsg.orientation);
    auto targetAngle = currentAngle + angle_increment_degree * M_PI / 180.0;
    goal.pose.pose.position = currentPoseMsg.position;
    tf2::Quaternion q;
    q.setEuler(targetAngle, 0, 0);
    goal.pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::PoseStamped stampedCurrentPoseMsg;
    stampedCurrentPoseMsg.header.frame_id = referenceFrame;
    stampedCurrentPoseMsg.header.stamp = getNode()->now();
    stampedCurrentPoseMsg.pose = currentPoseMsg;

    this->requiresClient(moveBaseClient_);
    odomTracker->pushPath();

    odomTracker->setStartPoint(stampedCurrentPoseMsg);
    odomTracker->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);

    RCLCPP_INFO_STREAM(getNode()->get_logger(),"current pose: " << currentPoseMsg);
    RCLCPP_INFO_STREAM(getNode()->get_logger(),"goal pose: " << goal.pose.pose);
    moveBaseClient_->sendGoal(goal);
}

} // namespace cl_move_base_z
