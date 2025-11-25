// Copyright 2023 RobosoftAI Inc.
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
 * 	 Authors: Jaycee Lock & Brett Aldrich
 *
 *   Description: HTTP/HTTPS Client using pure component-based architecture
 *
 *   Architecture:
 *     ClHttp (minimal orchestrator) creates and configures three components:
 *       - CpHttpConnectionManager: io_context and thread management
 *       - CpHttpSessionManager: SSL context and session creation
 *       - CpHttpRequestExecutor: Request execution and response signals
 *
 *   Usage:
 *     Behaviors should access CpHttpRequestExecutor component directly via requiresComponent()
 *     and connect to its onResponseReceived_ signal for HTTP responses.
 *
 *   See README.md for complete documentation and examples.
 *
 ******************************************************************************************************************/

#pragma once

#include <http_client/components/cp_http_connection_manager.hpp>
#include <http_client/components/cp_http_request_executor.hpp>
#include <http_client/components/cp_http_session_manager.hpp>
#include <http_client/http_session_base.hpp>
#include <smacc2/smacc_client.hpp>
#include <string>

namespace cl_http
{
/**
 * @brief HTTP/HTTPS client with pure component-based architecture
 *
 * This client acts as a minimal orchestrator that creates and configures
 * three specialized components during initialization. All HTTP functionality
 * is delegated to components.
 *
 * Components created:
 *   - CpHttpConnectionManager: Manages io_context and worker thread
 *   - CpHttpSessionManager: Manages SSL context and creates sessions
 *   - CpHttpRequestExecutor: Executes requests and emits response signals
 *
 * Behaviors should access CpHttpRequestExecutor directly via requiresComponent()
 * rather than using the client interface.
 */
class ClHttp : public smacc2::ISmaccClient
{
public:
  enum class kHttpRequestMethod
  {
    GET = static_cast<int>(boost::beast::http::verb::get),
    POST = static_cast<int>(boost::beast::http::verb::post),
    PUT = static_cast<int>(boost::beast::http::verb::put),
  };

  using TResponse = http_session_base::TResponse;

  explicit ClHttp(const std::string & server, const int & timeout = 1500);

  virtual ~ClHttp();

  void onInitialize() override;

  // Component composition during orthogonal initialization
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Create components
    connectionManager_ = this->createComponent<CpHttpConnectionManager, TOrthogonal, ClHttp>();
    sessionManager_ = this->createComponent<CpHttpSessionManager, TOrthogonal, ClHttp>();
    requestExecutor_ = this->createComponent<CpHttpRequestExecutor, TOrthogonal, ClHttp>();

    // Set component dependencies
    requestExecutor_->setDependencies(connectionManager_, sessionManager_);

    // Configure session manager with server URL
    sessionManager_->setServerUrl(server_url_);
  }

private:
  const int HTTP_VERSION = 11;

  bool initialized_;

  std::string server_url_;  // Server URL for component configuration

  // Component references
  CpHttpConnectionManager * connectionManager_;
  CpHttpSessionManager * sessionManager_;
  CpHttpRequestExecutor * requestExecutor_;
};
}  // namespace cl_http
