#include <move_base_z_client_plugin/client_behaviors/cb_undo_path_backwards.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>
#include <move_base_z_client_plugin/common.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

void CbUndoPathBackwards::onEntry()
{
    listener = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();

    nav_msgs::msg::Path forwardpath = odomTracker->getPath();
    //RCLCPP_INFO_STREAM(getNode()->get_logger(),"[UndoPathBackward] Current path backwards: " << forwardpath);

    odomTracker->setWorkingMode(WorkingMode::CLEAR_PATH);

    ClMoveBaseZ::Goal goal;
    // this line is used to flush/reset backward planner in the case it were already there
    //plannerSwitcher->setDefaultPlanners();
    if (forwardpath.poses.size() > 0)
    {
        goal.pose = forwardpath.poses.front();
        plannerSwitcher->setUndoPathBackwardPlanner();
        moveBaseClient_->sendGoal(goal);
    }
}

void CbUndoPathBackwards::onExit()
{
    auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();
    odomTracker->popPath();
}

} // namespace cl_move_base_z