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
#include <lifecyclenode_client/lifecyclenode_client.hpp>
#include <string>

namespace cl_lifecyclenode
{
ClLifecycleNode::ClLifecycleNode(std::string node_name) : nodeName_(node_name) {}

ClLifecycleNode::~ClLifecycleNode() {}

void ClLifecycleNode::onInitialize()
{
  client_get_state_ = this->getNode()->create_client<lifecycle_msgs::srv::GetState>(
    this->nodeName_ + node_get_state_topic);
  client_change_state_ = this->getNode()->create_client<lifecycle_msgs::srv::ChangeState>(
    this->nodeName_ + node_change_state_topic);

  this->subscription_transition_event_ =
    this->getNode() /*  */->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      this->nodeName_ + node_transition_event_topic, 100,
      std::bind(&ClLifecycleNode::onTransitionEvent, this, std::placeholders::_1));
}

void ClLifecycleNode::changeState(uint8_t state)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = state;
  auto future = client_change_state_->async_send_request(request);
  future.wait();
}

void ClLifecycleNode::activate()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

void ClLifecycleNode::deactivate()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

void ClLifecycleNode::configure()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

void ClLifecycleNode::cleanup()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

void ClLifecycleNode::shutdown()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

void ClLifecycleNode::onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  RCLCPP_INFO(
    this->getNode()->get_logger(), "[ClLifecycleNode] Transition event received: %d -> %d",
    msg->start_state.id, msg->goal_state.id);

  lastTransitionEvent_ = msg;
  // switch (msg->transition.id)
  // {
  // case lifecycle_msgs::msg::Transition::TRANSITION_CREATE:
  // if (msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN &&
  //     msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  // {
  //   RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_CREATE");
  //   this->postOnTransitionCreate_();
  // }
  // // case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
  // else
  if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_CONFIGURE");
    this->postOnTransitionConfigure_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_CLEANUP");
    this->postOnTransitionCleanup_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ACTIVATE");
    this->postOnTransitionActivate_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_DEACTIVATE");
    this->postOnTransitionDeactivate_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_UNCONFIGURED_SHUTDOWN");
    this->postOnTransitionUnconfiguredShutdown_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_INACTIVE_SHUTDOWN");
    this->postOnTransitionInactiveShutdown_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ACTIVE_SHUTDOWN");
    this->postOnTransitionActiveShutdown_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY:
  else if (msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  // &&msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DESTROYING

  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_DESTROY");
    this->postOnTransitionDestroy_();
  }

  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_SUCCESS:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_CONFIGURE_SUCCESS");
    this->postOnTransitionOnConfigureSuccess_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_FAILURE:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_CONFIGURE_FAILURE");
    this->postOnTransitionOnConfigureFailure_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR:
  if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_CONFIGURE_ERROR");
    this->postOnTransitionOnConfigureError_();
  }

  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_SUCCESS:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_CLEANUP_SUCCESS");
    this->postOnTransitionOnCleanupSuccess_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_FAILURE:
  // else if (msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP
  // &&msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  // {
  //   RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_CLEANUP_FAILURE");
  //   this->postOnTransitionOnCleanupFailure_();
  // }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_ERROR:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_CLEANUP_ERROR");
    this->postOnTransitionOnCleanupError_();
  }

  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_SUCCESS:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_ACTIVATE_SUCCESS");
    this->postOnTransitionOnActivateSuccess_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_FAILURE:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_ACTIVATE_FAILURE");
    this->postOnTransitionOnActivateFailure_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_ACTIVATE_ERROR");
    this->postOnTransitionOnActivateError_();
  }

  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_SUCCESS:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_DEACTIVATE_SUCCESS");
    this->postOnTransitionOnDeactivateSuccess_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_FAILURE:
  // else if (msg->start_state.id == &&msg->goal_state.id ==)
  // {
  //   RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_DEACTIVATE_FAILURE");
  //   this->postOnTransitionOnDeactivateFailure_();
  // }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_ERROR:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_DEACTIVATE_ERROR");
    this->postOnTransitionOnDeactivateError_();
  }

  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_SHUTDOWN_SUCCESS");
    this->postOnTransitionOnShutdownSuccess_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE:
  // else if (msg->start_state.id == &&msg->goal_state.id ==)
  // {
  //   RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_SHUTDOWN_FAILURE");
  //   this->postOnTransitionOnShutdownFailure_();
  // }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_ERROR:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_SHUTDOWN_ERROR");
    this->postOnTransitionOnShutdownError_();
  }

  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_SUCCESS:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_ERROR_SUCCESS");
    this->postOnTransitionOnErrorSuccess_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_FAILURE:
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_ERROR_FAILURE");
    this->postOnTransitionOnErrorFailure_();
  }
  // case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_ERROR:
  // else if (msg->start_state.id == &&msg->goal_state.id ==)
  // {
  //   RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_ON_ERROR_ERROR");
  //   this->postOnTransitionOnErrorError_();
  // }

  // default:
  else
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "TRANSITION_UNKNOWN");
  }
  // }

}  // namespace cl_lifecyclenode
}  // namespace cl_lifecyclenode
