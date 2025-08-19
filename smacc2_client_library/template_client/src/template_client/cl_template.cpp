// Copyright 2024 Your Organization
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

#include <template_client/cl_template.hpp>
#include <std_msgs/msg/string.hpp>

namespace cl_template
{
// ============================================================================
// Constructor
// ============================================================================
ClTemplate::ClTemplate(std::string topicName)
: topic_name_(topicName), 
  publish_rate_(1.0), 
  is_active_(false),
  processing_(false)
{
  RCLCPP_INFO(getLogger(), "[ClTemplate] Constructing with topic: %s", topic_name_.c_str());
}

// ============================================================================
// Destructor
// ============================================================================
ClTemplate::~ClTemplate()
{
  RCLCPP_INFO(getLogger(), "[ClTemplate] Destroying client");
  stopOperation();
}

// ============================================================================
// onInitialize - Called when the client is initialized by SMACC2
// ============================================================================
void ClTemplate::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[ClTemplate] Initializing client");
  
  // Get the node from SMACC2
  auto node = getNode();
  
  // Create publisher
  publisher_ = node->create_publisher<std_msgs::msg::String>(
    topic_name_ + "_out", 10);
  
  // Create subscriber with callback
  subscriber_ = node->create_subscription<std_msgs::msg::String>(
    topic_name_ + "_in", 10,
    std::bind(&ClTemplate::messageCallback, this, std::placeholders::_1));
  
  // Create a timer for periodic operations (optional)
  timer_ = node->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ClTemplate::timerCallback, this));
  
  // Validate configuration
  if (!validateConfiguration())
  {
    RCLCPP_WARN(getLogger(), "[ClTemplate] Configuration validation failed");
  }
  
  RCLCPP_INFO(getLogger(), "[ClTemplate] Initialization complete");
}

// ============================================================================
// Public Methods
// ============================================================================
void ClTemplate::startOperation()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (is_active_)
  {
    RCLCPP_WARN(getLogger(), "[ClTemplate] Operation already active");
    return;
  }
  
  RCLCPP_INFO(getLogger(), "[ClTemplate] Starting operation");
  is_active_ = true;
  processing_ = true;
  
  // Emit status change signal
  onStatusChanged_(1);  // 1 = active
  
  // Perform startup tasks
  updateInternalState();
}

void ClTemplate::stopOperation()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (!is_active_)
  {
    RCLCPP_WARN(getLogger(), "[ClTemplate] Operation already stopped");
    return;
  }
  
  RCLCPP_INFO(getLogger(), "[ClTemplate] Stopping operation");
  is_active_ = false;
  processing_ = false;
  
  // Emit status change signal
  onStatusChanged_(0);  // 0 = inactive
  
  // Cancel any pending operations
  if (timer_)
  {
    timer_->cancel();
  }
}

void ClTemplate::configureParameters()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  RCLCPP_INFO(getLogger(), "[ClTemplate] Configuring parameters");
  
  // Example: Update configuration
  // this->some_param_ = param_value;
  
  // Validate new configuration
  if (!validateConfiguration())
  {
    RCLCPP_ERROR(getLogger(), "[ClTemplate] Invalid configuration");
    // Restore previous configuration or handle error
  }
}

// ============================================================================
// Protected Callback Methods
// ============================================================================
void ClTemplate::messageCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!is_active_)
  {
    RCLCPP_DEBUG(getLogger(), "[ClTemplate] Ignoring message - client not active");
    return;
  }
  
  RCLCPP_DEBUG(getLogger(), "[ClTemplate] Received message: %s", msg->data.c_str());
  
  // Process the message
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Update internal state based on message
    // ... your processing logic here ...
  }
  
  // Emit data received signal
  onDataReceived_();
  
  // Example: Publish response
  auto response_msg = std::make_shared<std_msgs::msg::String>();
  response_msg->data = "Processed: " + msg->data;
  publisher_->publish(*response_msg);
}

void ClTemplate::timerCallback()
{
  if (!is_active_)
  {
    return;
  }
  
  RCLCPP_DEBUG(getLogger(), "[ClTemplate] Timer callback triggered");
  
  // Perform periodic tasks
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Example: Publish status message
  auto status_msg = std::make_shared<std_msgs::msg::String>();
  status_msg->data = "Status: " + std::to_string(is_active_);
  publisher_->publish(*status_msg);
  
  // Update internal state
  updateInternalState();
}

// ============================================================================
// Private Methods
// ============================================================================
void ClTemplate::updateInternalState()
{
  // Internal state update logic
  RCLCPP_DEBUG(getLogger(), "[ClTemplate] Updating internal state");
  
  // Example state machine or processing logic
  if (processing_)
  {
    // Perform processing steps
    // ...
  }
}

bool ClTemplate::validateConfiguration()
{
  // Validate current configuration
  bool valid = true;
  
  if (topic_name_.empty())
  {
    RCLCPP_ERROR(getLogger(), "[ClTemplate] Topic name cannot be empty");
    valid = false;
  }
  
  if (publish_rate_ <= 0)
  {
    RCLCPP_ERROR(getLogger(), "[ClTemplate] Publish rate must be positive");
    valid = false;
  }
  
  // Add more validation as needed
  
  return valid;
}

// ============================================================================
// Action-Based Client Implementation (commented out example)
// ============================================================================
/*
template <typename TAction>
ClTemplateAction<TAction>::ClTemplateAction(std::string actionName)
: Base(actionName), action_name_(actionName)
{
  RCLCPP_INFO(this->getLogger(), "[ClTemplateAction] Constructing with action: %s", 
              action_name_.c_str());
}

template <typename TAction>
ClTemplateAction<TAction>::~ClTemplateAction()
{
  RCLCPP_INFO(this->getLogger(), "[ClTemplateAction] Destroying client");
}

template <typename TAction>
void ClTemplateAction<TAction>::onInitialize()
{
  // Call base class initialization
  Base::onInitialize();
  
  RCLCPP_INFO(this->getLogger(), "[ClTemplateAction] Custom initialization");
  
  // Add custom initialization here
}

template <typename TAction>
void ClTemplateAction<TAction>::sendCustomGoal()
{
  RCLCPP_INFO(this->getLogger(), "[ClTemplateAction] Sending custom goal");
  
  // Create goal message
  typename TAction::Goal goal;
  // ... populate goal ...
  
  // Send goal using base class method
  this->sendGoal(goal);
}

template <typename TAction>
void ClTemplateAction<TAction>::cancelCurrentGoal()
{
  RCLCPP_INFO(this->getLogger(), "[ClTemplateAction] Cancelling current goal");
  
  // Cancel using base class method
  this->cancelGoal();
}
*/

}  // namespace cl_template