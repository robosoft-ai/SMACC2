// Copyright 2024 RobosoftAI Inc.
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

#pragma once

#include <smacc2/component.hpp>
#include <smacc2/smacc_default_events.hpp>
#include <smacc2/smacc_signal.hpp>
#include <smacc2/smacc_state_machine.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace smacc2
{
namespace client_core_components
{
using namespace smacc2::default_events;

/**
 * @brief Generic ROS2 service client component for SMACC2
 *
 * CpServiceClient provides a reusable service client component that can be used
 * by any SMACC2 client library requiring ROS2 service communication. It follows
 * the same pattern as CpActionClient, providing signals, event posting, and
 * thread-safe service call management.
 *
 * Key Features:
 * - Template-based for type safety with any ROS2 service type
 * - SMACC2 signal integration for event-driven responses
 * - Automatic SMACC2 event posting via template configuration
 * - Both synchronous and asynchronous service call support
 * - Thread-safe service call execution
 * - Service availability checking and waiting
 *
 * Usage Example:
 * @code
 * // In your client's onInitialize():
 * auto serviceClient = this->createComponent<
 *   smacc2::client_core_components::CpServiceClient<my_msgs::srv::MyService>>(
 *   "/my_service_name");
 *
 * // Send a request:
 * auto request = std::make_shared<my_msgs::srv::MyService::Request>();
 * request->data = 42;
 * serviceClient->sendRequestSync(request);
 * @endcode
 *
 * @tparam ServiceType ROS2 service type (e.g., std_srvs::srv::SetBool)
 */
template <typename ServiceType>
class CpServiceClient : public smacc2::ISmaccComponent
{
public:
  // Type aliases for cleaner code
  using Client = rclcpp::Client<ServiceType>;
  using Request = typename ServiceType::Request;
  using Response = typename ServiceType::Response;
  using SharedRequest = typename Request::SharedPtr;
  using SharedResponse = typename Response::SharedPtr;
  using SharedFuture = typename rclcpp::Client<ServiceType>::SharedFuture;

  // Configuration options (set before onInitialize() or via constructor)
  std::optional<std::string> serviceName;
  std::optional<std::chrono::milliseconds> serviceTimeout;

  // SMACC2 Signals for component communication
  // These signals are emitted when service events occur, allowing other
  // components and behaviors to react to service responses
  smacc2::SmaccSignal<void(const SharedResponse &)> onServiceResponse_;
  smacc2::SmaccSignal<void()> onServiceRequestSent_;
  smacc2::SmaccSignal<void()> onServiceFailure_;

  // Event posting functions (configured during component initialization)
  // These lambdas are set up with proper template parameters to post
  // type-safe SMACC2 events to the state machine
  std::function<void(const SharedResponse &)> postResponseEvent;
  std::function<void()> postRequestSentEvent;
  std::function<void()> postFailureEvent;

  /**
   * @brief Default constructor
   */
  CpServiceClient() = default;

  /**
   * @brief Constructor with service name
   * @param serviceName ROS2 service name (e.g., "/my_node/my_service")
   */
  CpServiceClient(const std::string & serviceName) : serviceName(serviceName) {}

  /**
   * @brief Constructor with service name and timeout
   * @param serviceName ROS2 service name
   * @param serviceTimeout Maximum time to wait for service response
   */
  CpServiceClient(const std::string & serviceName, std::chrono::milliseconds serviceTimeout)
  : serviceName(serviceName), serviceTimeout(serviceTimeout)
  {
  }

  virtual ~CpServiceClient() = default;

  /**
   * @brief Send a service request asynchronously
   *
   * Sends a service request and returns a future. The caller can wait on
   * the future or let the component handle the response via signals/events.
   *
   * @param request Shared pointer to the service request
   * @return SharedFuture Future for the service response
   */
  SharedFuture sendRequest(SharedRequest request)
  {
    std::lock_guard<std::mutex> lock(serviceMutex_);

    if (!client_)
    {
      RCLCPP_ERROR_STREAM(
        getLogger(), "[" << this->getName() << "] Service client not initialized!");
      throw std::runtime_error("Service client not initialized");
    }

    if (!client_->service_is_ready())
    {
      RCLCPP_WARN_STREAM(
        getLogger(), "[" << this->getName() << "] Service not ready: " << *serviceName);
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Sending service request to: " << *serviceName);

    // Send the async request
    auto future = client_->async_send_request(request);
    lastRequest_ = future;

    // Emit signal that request was sent
    onServiceRequestSent_();
    if (postRequestSentEvent) postRequestSentEvent();

    return future;
  }

  /**
   * @brief Send a service request synchronously (blocking)
   *
   * Sends a service request and blocks until the response is received or
   * a timeout occurs. This is a convenience method that wraps sendRequest()
   * with a blocking wait.
   *
   * @param request Shared pointer to the service request
   * @return SharedResponse Response from the service
   * @throws std::runtime_error if service call fails or times out
   */
  SharedResponse sendRequestSync(SharedRequest request)
  {
    auto future = sendRequest(request);

    // Wait for response with timeout
    std::future_status status;
    if (serviceTimeout)
    {
      status = future.wait_for(*serviceTimeout);
      if (status == std::future_status::timeout)
      {
        RCLCPP_ERROR_STREAM(
          getLogger(), "[" << this->getName() << "] Service call timed out: " << *serviceName);
        onServiceFailure_();
        if (postFailureEvent) postFailureEvent();
        throw std::runtime_error("Service call timeout");
      }
    }
    else
    {
      future.wait();
    }

    // Get the response
    try
    {
      auto response = future.get();

      RCLCPP_INFO_STREAM(
        getLogger(),
        "[" << this->getName() << "] Service response received from: " << *serviceName);

      // Emit signals and events
      onServiceResponse_(response);
      if (postResponseEvent) postResponseEvent(response);

      return response;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(
        getLogger(),
        "[" << this->getName() << "] Service call failed: " << *serviceName << " - " << e.what());
      onServiceFailure_();
      if (postFailureEvent) postFailureEvent();
      throw;
    }
  }

  /**
   * @brief Check if the service server is ready
   * @return true if service is available, false otherwise
   */
  bool isServiceReady() const { return client_ && client_->service_is_ready(); }

  /**
   * @brief Wait for the service to become available
   *
   * Blocks until the service server is ready to accept requests.
   * Uses a default timeout or the configured serviceTimeout.
   */
  void waitForService()
  {
    if (client_)
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << this->getName() << "] Waiting for service: " << *serviceName);

      if (serviceTimeout)
      {
        auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(*serviceTimeout);
        if (!client_->wait_for_service(timeout))
        {
          RCLCPP_WARN_STREAM(
            getLogger(), "[" << this->getName() << "] Service wait timed out: " << *serviceName);
        }
      }
      else
      {
        client_->wait_for_service();
      }

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << this->getName() << "] Service ready: " << *serviceName);
    }
  }

  /**
   * @brief Component initialization - creates the ROS2 service client
   *
   * Called by SMACC2 framework during component initialization.
   * Creates the underlying rclcpp::Client for the specified service.
   */
  void onInitialize() override
  {
    if (!serviceName)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "[" << this->getName() << "] Service name not set!");
      return;
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Initializing service client for: " << *serviceName);

    client_ = getNode()->create_client<ServiceType>(*serviceName);

    RCLCPP_INFO_STREAM(
      getLogger(),
      "[" << this->getName() << "] DONE Initializing service client for: " << *serviceName);
  }

  /**
   * @brief Configure event posting with proper template parameters
   *
   * This method is called during component allocation to set up the event
   * posting lambdas with the correct orthogonal and source object types.
   * This enables type-safe event posting to the state machine.
   *
   * @tparam TOrthogonal The orthogonal type that owns this component
   * @tparam TSourceObject The source object type (typically the client)
   */
  template <typename TOrthogonal, typename TSourceObject>
  void onComponentInitialization()
  {
    // Set up event posting functions with proper template parameters
    postResponseEvent = [this](const SharedResponse & response)
    {
      auto * ev = new EvServiceResponse<TSourceObject, TOrthogonal, Response>();
      ev->response = *response;
      this->postEvent(ev);
      RCLCPP_DEBUG(getLogger(), "[%s] SERVICE RESPONSE EVENT", this->getName().c_str());
    };

    postRequestSentEvent = [this]()
    {
      auto * ev = new EvServiceRequestSent<TSourceObject, TOrthogonal>();
      this->postEvent(ev);
      RCLCPP_DEBUG(getLogger(), "[%s] SERVICE REQUEST SENT EVENT", this->getName().c_str());
    };

    postFailureEvent = [this]()
    {
      auto * ev = new EvServiceFailure<TSourceObject, TOrthogonal>();
      this->postEvent(ev);
      RCLCPP_DEBUG(getLogger(), "[%s] SERVICE FAILURE EVENT", this->getName().c_str());
    };
  }

  /**
   * @brief Helper method to connect a callback to the response signal
   *
   * Provides a convenient way to connect member functions to the service
   * response signal with proper lifetime management.
   *
   * @tparam T Type of the callback object
   * @param callback Member function pointer to call on response
   * @param object Object instance that owns the callback
   * @return Signal connection handle
   */
  template <typename T>
  boost::signals2::connection onResponse(void (T::*callback)(const SharedResponse &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onServiceResponse_, callback, object);
  }

  /**
   * @brief Helper method to connect a callback to the request sent signal
   *
   * @tparam T Type of the callback object
   * @param callback Member function pointer to call when request is sent
   * @param object Object instance that owns the callback
   * @return Signal connection handle
   */
  template <typename T>
  boost::signals2::connection onRequestSent(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onServiceRequestSent_, callback, object);
  }

  /**
   * @brief Helper method to connect a callback to the failure signal
   *
   * @tparam T Type of the callback object
   * @param callback Member function pointer to call on failure
   * @param object Object instance that owns the callback
   * @return Signal connection handle
   */
  template <typename T>
  boost::signals2::connection onFailure(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onServiceFailure_, callback, object);
  }

  /**
   * @brief Get the underlying ROS2 service client
   *
   * Provides access to the raw rclcpp::Client for advanced usage scenarios.
   *
   * @return Shared pointer to the rclcpp service client
   */
  std::shared_ptr<Client> getServiceClient() const { return client_; }

private:
  std::shared_ptr<Client> client_ = nullptr;
  std::optional<SharedFuture> lastRequest_;
  std::mutex serviceMutex_;
};

}  // namespace client_core_components
}  // namespace smacc2
