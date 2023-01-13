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
      this->nodeName_ + node_transition_event_topic,
      1', std::bind(&ClLifecycleNode::onTransitionEvent, this, std::placeholders::_1));
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
    this->getNode()->get_logger(), "[ClLifecycleNode] Transition event received: %s, %d",
    msg->transition.label.c_str(), msg->transition.id);

  lastTransitionEvent_ = msg;
  switch (msg->transition.id)
  {
    case lifecycle_msgs::msg::Transition::TRANSITION_CREATE:
      this->postOnTransitionCreate_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
      this->postOnTransitionConfigure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
      this->postOnTransitionCleanup_();

      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
      this->postOnTransitionActivate_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
      this->postOnTransitionDeactivate_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
      this->postOnTransitionUnconfiguredShutdown_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
      this->postOnTransitionInactiveShutdown_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
      this->postOnTransitionActiveShutdown_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY:
      this->postOnTransitionDestroy_();
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_SUCCESS:
      this->postOnTransitionOnConfigureSuccess_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_FAILURE:
      this->postOnTransitionOnConfigureFailure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR:
      this->postOnTransitionOnConfigureError_();
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_SUCCESS:
      this->postOnTransitionOnCleanupSuccess_();

      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_FAILURE:
      this->postOnTransitionOnCleanupFailure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_ERROR:
      this->postOnTransitionOnCleanupError_();
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_SUCCESS:
      this->postOnTransitionOnActivateSuccess_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_FAILURE:
      this->postOnTransitionOnActivateFailure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR:
      this->postOnTransitionOnActivateError_();
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_SUCCESS:
      this->postOnTransitionOnDeactivateSuccess_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_FAILURE:
      this->postOnTransitionOnDeactivateFailure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_ERROR:
      this->postOnTransitionOnDeactivateError_();
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS:
      this->postOnTransitionOnShutdownSuccess_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE:
      this->postOnTransitionOnShutdownFailure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_ERROR:
      this->postOnTransitionOnShutdownError_();
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_SUCCESS:
      this->postOnTransitionOnErrorSuccess_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_FAILURE:
      this->postOnTransitionOnErrorFailure_();
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_ERROR:
      this->postOnTransitionOnErrorError_();
      break;

    default:
      break;
  }

}  // namespace cl_lifecyclenode
}  // namespace cl_lifecyclenode
