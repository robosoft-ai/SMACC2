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
// #include <actionlib/server/simple_action_server.h>
// #include <dynamic_reconfigure/DoubleParameter.h>
// #include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/Config.h>

#include <functional>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sm_dance_bot_warehouse_3/action/led_control.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <thread>
// #include <sm_dance_bot_warehouse_3/action/LEDControlActionResult.h>
// #include <sm_dance_bot_warehouse_3/action/LEDControlResult.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// This class describes a preemptable-on/off tool action server to be used from smacc
// shows in rviz a sphere whose color describe the current state (unknown, running, idle)
class LEDActionServer : public rclcpp::Node
{
public:
  std::shared_ptr<rclcpp_action::Server<sm_dance_bot_warehouse_3::action::LEDControl>> as_;
  using GoalHandleLEDControl =
    rclcpp_action::ServerGoalHandle<sm_dance_bot_warehouse_3::action::LEDControl>;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stateMarkerPublisher_;

  uint8_t cmd;

  uint8_t currentState_;

  /**
******************************************************************************************************************
* constructor()
******************************************************************************************************************
*/
  LEDActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("led_action_server_node", options)
  {
    currentState_ = sm_dance_bot_warehouse_3::action::LEDControl::Result::STATE_UNKNOWN;
  }

  /**
******************************************************************************************************************
* execute()
******************************************************************************************************************
*/
  void execute(const std::shared_ptr<GoalHandleLEDControl>
                 gh)  // Note: "Action" is not appended to DoDishes here
  {
    auto goal = gh->get_goal();
    RCLCPP_INFO_STREAM(get_logger(), "Tool action server request: " << goal->command);

    cmd = goal->command;

    if (goal->command == sm_dance_bot_warehouse_3::action::LEDControl_Goal::CMD_ON)
    {
      RCLCPP_INFO(this->get_logger(), "ON");
      currentState_ = sm_dance_bot_warehouse_3::action::LEDControl_Result::STATE_RUNNING;
    }
    else if (goal->command == sm_dance_bot_warehouse_3::action::LEDControl_Goal::CMD_OFF)
    {
      RCLCPP_INFO(this->get_logger(), "OFF");
      currentState_ = sm_dance_bot_warehouse_3::action::LEDControl_Result::STATE_IDLE;
    }

    auto feedback_msg = std::make_shared<sm_dance_bot_warehouse_3::action::LEDControl::Feedback>();

    // 10Hz internal loop
    rclcpp::Rate rate(20);

    while (rclcpp::ok())
    {
      gh->publish_feedback(feedback_msg);

      publishStateMarker();
      rate.sleep();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 2000, "Loop feedback");
    }

    auto result = std::make_shared<sm_dance_bot_warehouse_3::action::LEDControl::Result>();
    result->state = this->currentState_;

    // never reach succeeded because were are interested in keeping the feedback alive
    //as_->setSucceeded();
    gh->succeed(result);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const sm_dance_bot_warehouse_3::action::LEDControl::Goal> /*goal*/)
  {
    // (void)uuid;
    // // Let's reject sequences that are over 9000
    // if (goal->order > 9000) {
    //   return rclcpp_action::GoalResponse::REJECT;
    // }

    // lets accept everything
    RCLCPP_INFO(this->get_logger(), "Handle goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLEDControl> /*goal_handle*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
    //(void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleLEDControl> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&LEDActionServer::execute, this, std::placeholders::_1), goal_handle}
      .detach();
  }

  /**
******************************************************************************************************************
* run()
******************************************************************************************************************
*/
  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Creating tool action server");
    //as_ = std::make_shared<Server>(n, "led_action_server", boost::bind(&LEDActionServer::execute, this,  _1), false);

    this->as_ = rclcpp_action::create_server<sm_dance_bot_warehouse_3::action::LEDControl>(
      this, "led_action_server",
      std::bind(&LEDActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LEDActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&LEDActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Starting Tool Action Server");
    stateMarkerPublisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("tool_markers", 1);
  }

  /**
******************************************************************************************************************
* publishStateMarker()
******************************************************************************************************************
*/
  void publishStateMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();

    marker.ns = "tool_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 1;

    if (currentState_ == sm_dance_bot_warehouse_3::action::LEDControl::Result::STATE_RUNNING)
    {
      // show green ball
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
    }
    else if (currentState_ == sm_dance_bot_warehouse_3::action::LEDControl::Result::STATE_IDLE)
    {
      // show gray ball
      marker.color.r = 0.7;
      marker.color.g = 0.7;
      marker.color.b = 0.7;
    }
    else
    {
      // show black ball
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 0;
    }

    marker.pose.orientation.w = 1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 1;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.push_back(marker);

    stateMarkerPublisher_->publish(ma);
  }
};

/**
******************************************************************************************************************
* main()
******************************************************************************************************************
*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ledactionserver = std::make_shared<LEDActionServer>();
  ledactionserver->run();

  rclcpp::spin(ledactionserver);
  rclcpp::shutdown();
  return 0;
}
