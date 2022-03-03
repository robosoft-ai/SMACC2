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
#include <actionlib/server/simple_action_server.h>
#include <odom_tracker/OdomTrackerAction.h>
#include <memory>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>

typedef actionlib::SimpleActionServer<odom_tracker::OdomTrackerAction> Server;

using namespace odom_tracker;
using namespace cl_nav2z::odom_tracker;

class OdomTrackerActionServer
{
public:
  std::shared_ptr<Server> as_;
  OdomTracker odomTracker;

  OdomTrackerActionServer() : odomTracker("move_base") {}

  /**
******************************************************************************************************************
* execute()
******************************************************************************************************************
*/
  void execute(
    const OdomTrackerGoalConstPtr & goal)  // Note: "Action" is not appended to DoDishes here
  {
    try
    {
      switch (goal->command)
      {
        case OdomTrackerGoal::RECORD_PATH:
          odomTracker.setWorkingMode(WorkingMode::RECORD_PATH);
          break;

        case OdomTrackerGoal::CLEAR_PATH:
          odomTracker.setWorkingMode(WorkingMode::CLEAR_PATH);
          break;

        case OdomTrackerGoal::IDLE:
          odomTracker.setWorkingMode(WorkingMode::IDLE);
          break;

        case OdomTrackerGoal::START_BROADCAST_PATH:
          odomTracker.setPublishMessages(true);
          break;

        case OdomTrackerGoal::STOP_BROADCAST_PATH:
          odomTracker.setPublishMessages(false);
          break;

        case OdomTrackerGoal::PUSH_PATH:
          odomTracker.pushPath();
          break;

        case OdomTrackerGoal::POP_PATH:
          odomTracker.popPath();
          break;

        default:

          RCLCPP_ERROR(
            getLogger(), "Odom Tracker Node - Action Server execute error: incorrect command - %d",
            goal->command);
          as_->setAborted();
      }

      // never reach succeeded because were are interested in keeping the feedback alive
      as_->setSucceeded();
    }
    catch (std::exception & ex)
    {
      RCLCPP_ERROR(getLogger(), "Odom Tracker Node - Action Server execute error: %s", ex.what());
      as_->setAborted();
    }
  }

  /**
******************************************************************************************************************
* run()
******************************************************************************************************************
*/
  void run()
  {
    rclcpp::Node::SharedPtr n;
    RCLCPP_INFO(getLogger(), "Creating odom tracker action server");

    as_ = std::make_shared<Server>(
      n, "odom_tracker", boost::bind(&OdomTrackerActionServer::execute, this, _1), false);
    RCLCPP_INFO(getLogger(), "Starting OdomTracker Action Server");

    as_->start();

    ros::spin();
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "odom_tracker_node");
  OdomTrackerActionServer as;

  as.run();
}
