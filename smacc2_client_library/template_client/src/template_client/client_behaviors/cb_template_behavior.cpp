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

#include <template_client/client_behaviors/cb_template_behavior.hpp>

namespace cl_template
{
// ============================================================================
// CbTemplateBehavior Implementation
// ============================================================================
CbTemplateBehavior::CbTemplateBehavior()
: is_active_(false), retry_count_(0)
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Default constructor");
}

CbTemplateBehavior::CbTemplateBehavior(const CbTemplateBehaviorOptions& opts)
: options(opts), is_active_(false), retry_count_(0)
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Constructor with options");
}

CbTemplateBehavior::~CbTemplateBehavior()
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Destructor");
}

void CbTemplateBehavior::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] onEntry - Starting behavior");
  
  // Get the client from the orthogonal
  requiresClient(templateClient_);
  
  if (!templateClient_)
  {
    RCLCPP_ERROR(getLogger(), "[CbTemplateBehavior] Failed to get template client");
    if (postFailureEvent_)
    {
      postFailureEvent_();
    }
    return;
  }
  
  // Setup signal connections
  setupConnections();
  
  // Start timeout timer if configured
  if (options.timeout_seconds.has_value())
  {
    startTimeoutTimer();
  }
  
  // Mark as active
  is_active_ = true;
  retry_count_ = 0;
  
  // Execute the main action
  executeCustomAction();
}

void CbTemplateBehavior::onExit()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] onExit - Stopping behavior");
  
  is_active_ = false;
  
  // Stop timeout timer
  stopTimeoutTimer();
  
  // Teardown connections
  teardownConnections();
  
  // Cancel any ongoing operations
  cancelCurrentOperation();
}

void CbTemplateBehavior::configure(const CbTemplateBehaviorOptions& opts)
{
  options = opts;
  RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Configuration updated");
}

void CbTemplateBehavior::executeCustomAction()
{
  if (!is_active_ || !templateClient_)
  {
    RCLCPP_WARN(getLogger(), "[CbTemplateBehavior] Cannot execute - behavior not active or client not available");
    return;
  }
  
  RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Executing custom action (attempt %d/%d)",
              retry_count_ + 1, options.max_retries.value_or(1));
  
  // Start the client operation
  templateClient_->startOperation();
  
  // The actual work would be done in the client
  // This behavior just orchestrates and monitors
}

void CbTemplateBehavior::cancelCurrentOperation()
{
  if (templateClient_)
  {
    RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Cancelling current operation");
    templateClient_->stopOperation();
  }
}

void CbTemplateBehavior::onTimeout()
{
  RCLCPP_WARN(getLogger(), "[CbTemplateBehavior] Operation timed out");
  
  if (shouldRetry())
  {
    retry_count_++;
    RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Retrying operation...");
    executeCustomAction();
  }
  else
  {
    RCLCPP_ERROR(getLogger(), "[CbTemplateBehavior] Max retries reached or retry not allowed");
    if (postFailureEvent_)
    {
      postFailureEvent_();
    }
  }
}

void CbTemplateBehavior::onDataReceived()
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Data received from client");
  
  // Process received data
  // In this example, we consider data reception as success
  if (postSuccessEvent_)
  {
    RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Operation successful");
    postSuccessEvent_();
  }
}

void CbTemplateBehavior::onStatusChanged(int status)
{
  RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Client status changed to: %d", status);
  
  // Handle different status values
  if (status == 0)  // Inactive
  {
    RCLCPP_WARN(getLogger(), "[CbTemplateBehavior] Client became inactive");
  }
  else if (status == 1)  // Active
  {
    RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Client is active");
  }
}

bool CbTemplateBehavior::shouldRetry()
{
  return options.max_retries.has_value() && 
         retry_count_ < options.max_retries.value();
}

void CbTemplateBehavior::setupConnections()
{
  if (!templateClient_)
    return;
  
  RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Setting up signal connections");
  
  // Connect to client signals
  data_received_connection_ = templateClient_->onDataReceived_.connect(
    std::bind(&CbTemplateBehavior::onDataReceived, this));
  
  status_changed_connection_ = templateClient_->onStatusChanged_.connect(
    std::bind(&CbTemplateBehavior::onStatusChanged, this, std::placeholders::_1));
}

void CbTemplateBehavior::teardownConnections()
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Tearing down signal connections");
  
  data_received_connection_.disconnect();
  status_changed_connection_.disconnect();
}

void CbTemplateBehavior::startTimeoutTimer()
{
  if (!options.timeout_seconds.has_value())
    return;
  
  auto timeout_duration = std::chrono::duration<double>(options.timeout_seconds.value());
  
  RCLCPP_INFO(getLogger(), "[CbTemplateBehavior] Starting timeout timer: %.2f seconds", 
              options.timeout_seconds.value());
  
  timeout_timer_ = getNode()->create_wall_timer(
    timeout_duration,
    [this]() {
      this->onTimeout();
      this->timeout_timer_->cancel();  // One-shot timer
    });
}

void CbTemplateBehavior::stopTimeoutTimer()
{
  if (timeout_timer_)
  {
    RCLCPP_DEBUG(getLogger(), "[CbTemplateBehavior] Stopping timeout timer");
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }
}

// ============================================================================
// CbTemplateAsyncBehavior Implementation
// ============================================================================
CbTemplateAsyncBehavior::CbTemplateAsyncBehavior()
: cancel_requested_(false)
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateAsyncBehavior] Constructor");
}

void CbTemplateAsyncBehavior::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateAsyncBehavior] onEntry - Starting async behavior");
  
  // Call base class onEntry
  CbTemplateBehavior::onEntry();
  
  // Start async operation
  cancel_requested_ = false;
  startAsyncOperation();
}

void CbTemplateAsyncBehavior::onExit()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateAsyncBehavior] onExit - Stopping async behavior");
  
  // Request cancellation
  cancel_requested_ = true;
  
  // Notify async thread
  async_cv_.notify_all();
  
  // Wait for async thread to finish
  if (async_thread_.joinable())
  {
    async_thread_.join();
  }
  
  // Call base class onExit
  CbTemplateBehavior::onExit();
}

void CbTemplateAsyncBehavior::startAsyncOperation()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateAsyncBehavior] Starting async operation");
  
  async_thread_ = std::thread([this]() {
    // Simulate async work
    std::unique_lock<std::mutex> lock(async_mutex_);
    
    // Wait for some condition or timeout
    bool timeout = !async_cv_.wait_for(lock, std::chrono::seconds(5),
                                       [this] { return cancel_requested_.load(); });
    
    if (cancel_requested_)
    {
      RCLCPP_INFO(getLogger(), "[CbTemplateAsyncBehavior] Async operation cancelled");
      onAsyncOperationComplete(false);
    }
    else if (timeout)
    {
      RCLCPP_INFO(getLogger(), "[CbTemplateAsyncBehavior] Async operation completed");
      onAsyncOperationComplete(true);
    }
  });
}

void CbTemplateAsyncBehavior::onAsyncOperationComplete(bool success)
{
  RCLCPP_INFO(getLogger(), "[CbTemplateAsyncBehavior] Async operation complete: %s",
              success ? "success" : "failure");
  
  if (success && postSuccessEvent_)
  {
    postSuccessEvent_();
  }
  else if (!success && postFailureEvent_)
  {
    postFailureEvent_();
  }
}

// ============================================================================
// CbTemplatePeriodicBehavior Implementation
// ============================================================================
CbTemplatePeriodicBehavior::CbTemplatePeriodicBehavior(double period_seconds)
: period_seconds_(period_seconds), is_paused_(false), iteration_count_(0)
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplatePeriodicBehavior] Constructor with period: %.2f seconds", 
               period_seconds);
}

CbTemplatePeriodicBehavior::~CbTemplatePeriodicBehavior()
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplatePeriodicBehavior] Destructor");
}

void CbTemplatePeriodicBehavior::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbTemplatePeriodicBehavior] onEntry - Starting periodic behavior");
  
  // Get the client
  requiresClient(templateClient_);
  
  if (!templateClient_)
  {
    RCLCPP_ERROR(getLogger(), "[CbTemplatePeriodicBehavior] Failed to get template client");
    return;
  }
  
  // Reset state
  is_paused_ = false;
  iteration_count_ = 0;
  
  // Create periodic timer
  auto period_duration = std::chrono::duration<double>(period_seconds_);
  periodic_timer_ = getNode()->create_wall_timer(
    period_duration,
    std::bind(&CbTemplatePeriodicBehavior::onPeriodicCallback, this));
}

void CbTemplatePeriodicBehavior::onExit()
{
  RCLCPP_INFO(getLogger(), "[CbTemplatePeriodicBehavior] onExit - Stopping periodic behavior");
  
  if (periodic_timer_)
  {
    periodic_timer_->cancel();
    periodic_timer_.reset();
  }
}

void CbTemplatePeriodicBehavior::onPeriodicCallback()
{
  if (is_paused_)
  {
    RCLCPP_DEBUG(getLogger(), "[CbTemplatePeriodicBehavior] Skipping iteration - paused");
    return;
  }
  
  iteration_count_++;
  RCLCPP_DEBUG(getLogger(), "[CbTemplatePeriodicBehavior] Periodic callback - iteration %d", 
               iteration_count_);
  
  // Perform periodic work
  if (templateClient_)
  {
    // Example: Configure client with iteration-specific parameters
    // templateClient_->configureParameters(...);
  }
  
  // Post periodic event
  if (postPeriodicEvent_)
  {
    postPeriodicEvent_();
  }
}

void CbTemplatePeriodicBehavior::setPeriod(double period_seconds)
{
  period_seconds_ = period_seconds;
  
  if (periodic_timer_)
  {
    // Recreate timer with new period
    periodic_timer_->cancel();
    auto period_duration = std::chrono::duration<double>(period_seconds_);
    periodic_timer_ = getNode()->create_wall_timer(
      period_duration,
      std::bind(&CbTemplatePeriodicBehavior::onPeriodicCallback, this));
  }
}

void CbTemplatePeriodicBehavior::pause()
{
  RCLCPP_INFO(getLogger(), "[CbTemplatePeriodicBehavior] Pausing periodic behavior");
  is_paused_ = true;
}

void CbTemplatePeriodicBehavior::resume()
{
  RCLCPP_INFO(getLogger(), "[CbTemplatePeriodicBehavior] Resuming periodic behavior");
  is_paused_ = false;
}

// ============================================================================
// CbTemplateConditionalBehavior Implementation
// ============================================================================
CbTemplateConditionalBehavior::CbTemplateConditionalBehavior(ConditionFunction condition)
: condition_(condition), condition_met_(false), timeout_(30)
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateConditionalBehavior] Constructor");
}

CbTemplateConditionalBehavior::~CbTemplateConditionalBehavior()
{
  RCLCPP_DEBUG(getLogger(), "[CbTemplateConditionalBehavior] Destructor");
}

void CbTemplateConditionalBehavior::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateConditionalBehavior] onEntry - Starting conditional behavior");
  
  // Get the client
  requiresClient(templateClient_);
  
  if (!templateClient_)
  {
    RCLCPP_ERROR(getLogger(), "[CbTemplateConditionalBehavior] Failed to get template client");
    if (postConditionFailedEvent_)
    {
      postConditionFailedEvent_();
    }
    return;
  }
  
  // Initialize state
  condition_met_ = false;
  start_time_ = std::chrono::steady_clock::now();
  
  // Start checking condition periodically
  check_timer_ = getNode()->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CbTemplateConditionalBehavior::checkCondition, this));
}

void CbTemplateConditionalBehavior::onExit()
{
  RCLCPP_INFO(getLogger(), "[CbTemplateConditionalBehavior] onExit - Stopping conditional behavior");
  
  if (check_timer_)
  {
    check_timer_->cancel();
    check_timer_.reset();
  }
}

void CbTemplateConditionalBehavior::setCondition(ConditionFunction condition)
{
  condition_ = condition;
}

void CbTemplateConditionalBehavior::checkCondition()
{
  if (!condition_)
  {
    RCLCPP_ERROR(getLogger(), "[CbTemplateConditionalBehavior] No condition function set");
    if (postConditionFailedEvent_)
    {
      postConditionFailedEvent_();
    }
    check_timer_->cancel();
    return;
  }
  
  // Check timeout
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > timeout_)
  {
    RCLCPP_WARN(getLogger(), "[CbTemplateConditionalBehavior] Condition check timed out");
    if (postConditionFailedEvent_)
    {
      postConditionFailedEvent_();
    }
    check_timer_->cancel();
    return;
  }
  
  // Evaluate condition
  if (condition_())
  {
    RCLCPP_INFO(getLogger(), "[CbTemplateConditionalBehavior] Condition met");
    condition_met_ = true;
    if (postConditionMetEvent_)
    {
      postConditionMetEvent_();
    }
    check_timer_->cancel();
  }
  else
  {
    RCLCPP_DEBUG(getLogger(), "[CbTemplateConditionalBehavior] Condition not yet met");
  }
}

}  // namespace cl_template