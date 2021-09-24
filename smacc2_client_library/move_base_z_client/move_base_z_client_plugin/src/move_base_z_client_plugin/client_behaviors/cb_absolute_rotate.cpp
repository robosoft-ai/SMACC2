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

#include <move_base_z_client_plugin/client_behaviors/cb_absolute_rotate.hpp>
#include <move_base_z_client_plugin/common.hpp>
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.hpp>
#include <move_base_z_client_plugin/components/pose/cp_pose.hpp>
#include <move_base_z_client_plugin/move_base_z_client_plugin.hpp>

#include <rclcpp/parameter_client.hpp>

namespace cl_move_base_z
{
CbAbsoluteRotate::CbAbsoluteRotate() {}

CbAbsoluteRotate::CbAbsoluteRotate(float absoluteGoalAngleDegree, float yaw_goal_tolerance)
{
  this->absoluteGoalAngleDegree = absoluteGoalAngleDegree;

  if (yaw_goal_tolerance >= 0)
  {
    this->yawGoalTolerance = yaw_goal_tolerance;
  }
}

void CbAbsoluteRotate::onEntry()
{
  double goal_angle;

  listener = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
  if (!this->absoluteGoalAngleDegree)
  {
    goal_angle = 0;
    this->getCurrentState()->param("goal_angle", goal_angle);
  }
  else
  {
    goal_angle = *this->absoluteGoalAngleDegree;
  }
  RCLCPP_INFO_STREAM(getLogger(), "[CbAbsoluteRotate] Absolute yaw Angle:" << goal_angle);

  auto plannerSwitcher = this->moveBaseClient_->getComponent<PlannerSwitcher>();
  // this should work better with a coroutine and await
  // this->plannerSwitcher_->setForwardPlanner();

  if (spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning)
  {
    plannerSwitcher->setPureSpinningPlanner();
  }
  else
  {
    plannerSwitcher->setDefaultPlanners();
  }

  updateTemporalBehaviorParameters(false);

  auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();

  ClMoveBaseZ::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();

  auto targetAngle = goal_angle * M_PI / 180.0;
  goal.pose.pose.position = currentPoseMsg.position;
  tf2::Quaternion q;
  q.setRPY(0, 0, targetAngle);
  goal.pose.pose.orientation = tf2::toMsg(q);

  auto odomTracker_ = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
  if (odomTracker_ != nullptr)
  {
    odomTracker_->pushPath();
    odomTracker_->setStartPoint(p->toPoseStampedMsg());
    odomTracker_->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);
  }

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("absolute_rotate_goal_checker");

  RCLCPP_INFO_STREAM(
    getLogger(),
    "[CbAbsoluteRotate] current pose yaw: " << tf2::getYaw(currentPoseMsg.orientation));
  RCLCPP_INFO_STREAM(
    getLogger(), "[CbAbsoluteRotate] goal pose yaw: " << tf2::getYaw(goal.pose.pose.orientation));
  moveBaseClient_->sendGoal(goal);
}

void CbAbsoluteRotate::updateTemporalBehaviorParameters(bool undo)
{
  auto log = this->getLogger();

  std::string nodename = "/controller_server";

  auto parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(this->getNode(), nodename);

  RCLCPP_INFO_STREAM(
    log, "[CbAbsoluteRotate] using a parameter client to update some controller parameters: "
           << nodename << ". Waiting service.");
  parameters_client->wait_for_service();

  RCLCPP_INFO_STREAM(log, "[CbAbsoluteRotate] Service found: " << nodename << ".");

  std::string localPlannerName;
  std::vector<rclcpp::Parameter> parameters;

  // dynamic_reconfigure::DoubleParameter yaw_goal_tolerance;
  rclcpp::Parameter yaw_goal_tolerance("goal_checker.yaw_goal_tolerance");

  rclcpp::Parameter max_vel_theta, min_vel_theta;

  bool isRosBasePlanner = !spinningPlanner || *spinningPlanner == SpiningPlanner::Default;
  bool isPureSpinningPlanner = spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning;

  // SELECTING CONTROLLER AND PARAMETERS NAME
  if (isPureSpinningPlanner)
  {
    localPlannerName = "PureSpinningLocalPlanner";
    max_vel_theta = rclcpp::Parameter(localPlannerName + ".max_angular_z_speed");
    min_vel_theta = rclcpp::Parameter(localPlannerName + ".min_vel_theta");
  }
  else if (isRosBasePlanner)
  {
    localPlannerName = "FollowPath";
    max_vel_theta = rclcpp::Parameter(localPlannerName + ".max_vel_theta");
    min_vel_theta = rclcpp::Parameter(localPlannerName + ".min_vel_theta");
  }

  if (!undo)
  {
    if (yawGoalTolerance)
    {
      // save old yaw tolerance
      auto fut = parameters_client->get_parameters(
        {localPlannerName + ".yaw_goal_tolerance"}, [&](auto futureParameters) {
          auto params = futureParameters.get();
          oldYawTolerance = params[0].as_double();
        });

      // make synchronous
      fut.get();

      yaw_goal_tolerance = rclcpp::Parameter("goal_checker.yaw_goal_tolerance", *yawGoalTolerance);
      parameters.push_back(yaw_goal_tolerance);
      RCLCPP_INFO(
        getLogger(),
        "[CbAbsoluteRotate] updating yaw tolerance local planner to: %lf, from previous value: "
        "%lf ",
        *yawGoalTolerance, this->oldYawTolerance);
    }

    if (maxVelTheta)
    {
      if (isRosBasePlanner)
      {
        // nh.getParam(nodename + "/"  + localPlannerName+"/min_vel_theta", oldMinVelTheta);
        // getCurrentState()->getParam(nodename + "/" + localPlannerName + "/max_vel_theta", oldMaxVelTheta);

        // save old yaw tolerance
        auto fut = parameters_client->get_parameters(
          {localPlannerName + ".max_vel_theta"}, [&](auto futureParameters) {
            auto params = futureParameters.get();
            oldMaxVelTheta = params[0].as_double();
          });

        // make synchronous
        fut.get();
      }

      max_vel_theta = rclcpp::Parameter(localPlannerName + ".max_vel_theta", *maxVelTheta);
      min_vel_theta = rclcpp::Parameter(localPlannerName + ".min_vel_theta", -*maxVelTheta);
      parameters.push_back(max_vel_theta);
      parameters.push_back(min_vel_theta);

      RCLCPP_INFO(
        log,
        "[CbAbsoluteRotate] updating max vel theta local planner to: %lf, from previous value: "
        "%lf ",
        *maxVelTheta, this->oldMaxVelTheta);
      RCLCPP_INFO(
        log,
        "[CbAbsoluteRotate] updating min vel theta local planner to: %lf, from previous value: "
        "%lf ",
        -*maxVelTheta, this->oldMinVelTheta);
    }
  }
  else
  {
    if (yawGoalTolerance)
    {
      yaw_goal_tolerance = rclcpp::Parameter("goal_checker.yaw_goal_tolerance", oldYawTolerance);
      RCLCPP_INFO(
        log,
        "[CbAbsoluteRotate] restoring yaw tolerance local planner from: %lf, to previous value: "
        "%lf ",
        *yawGoalTolerance, this->oldYawTolerance);
    }

    if (maxVelTheta)
    {
      if (isRosBasePlanner)
      {
        max_vel_theta = rclcpp::Parameter(localPlannerName + ".max_vel_theta", oldMaxVelTheta);
        min_vel_theta = rclcpp::Parameter(localPlannerName + ".max_vel_theta", oldMinVelTheta);
      }

      parameters.push_back(max_vel_theta);
      parameters.push_back(min_vel_theta);
      RCLCPP_INFO(
        log,
        "[CbAbsoluteRotate] restoring max vel theta local planner from: %lf, to previous value: "
        "%lf ",
        *maxVelTheta, this->oldMaxVelTheta);
      RCLCPP_INFO(
        log,
        "[CbAbsoluteRotate] restoring min vel theta local planner from: %lf, to previous value: "
        "%lf ",
        -(*maxVelTheta), this->oldMinVelTheta);
    }
  }

  if (parameters.size() > 0)
  {
    RCLCPP_INFO(log, "[CbAbsoluteRotate] parameters to update:  ");
    for (auto & p : parameters)
    {
      RCLCPP_INFO_STREAM(log, "[CbAbsoluteRotate] - " << p.get_name());
    }
  }

  auto futureResults = parameters_client->set_parameters(parameters);

  int i = 0;
  for (auto & res : futureResults.get())
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[CbAbsoluteRotate] parameter result: " << parameters[i].get_name() << "="
                                                           << parameters[i].as_string()
                                                           << ". Result: " << res.successful);
    i++;
  }

  RCLCPP_INFO(log, "[CbAbsoluteRotate] parameters updated");
}

void CbAbsoluteRotate::onExit()
{
  if (spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning)
  {
  }
  else
  {
  }

  this->updateTemporalBehaviorParameters(true);
}

}  // namespace cl_move_base_z
