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

// HTTP Request Executor Component - Executes HTTP requests and coordinates responses
// Authors: Claude (Anthropic AI)

#pragma once

#include <boost/beast/http.hpp>
#include <http_client/http_session_base.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>
#include <string>
#include <unordered_map>

namespace cl_http
{

// Forward declarations
class CpHttpConnectionManager;
class CpHttpSessionManager;

// Coordinates HTTP request execution and emits response signals
// Acquires strand and session from other components
class CpHttpRequestExecutor : public smacc2::ISmaccComponent
{
public:
  // HTTP request methods
  enum class HttpMethod
  {
    GET = static_cast<int>(boost::beast::http::verb::get),
    POST = static_cast<int>(boost::beast::http::verb::post),
    PUT = static_cast<int>(boost::beast::http::verb::put),
  };

  using TResponse = http_session_base::TResponse;

  CpHttpRequestExecutor();
  virtual ~CpHttpRequestExecutor();

  void onInitialize() override;

  // Set component dependencies (called by client during initialization)
  void setDependencies(CpHttpConnectionManager * connMgr, CpHttpSessionManager * sessMgr)
  {
    connectionManager_ = connMgr;
    sessionManager_ = sessMgr;
  }

  // Execute HTTP request with specified method, path, body, and headers
  void executeRequest(
    const HttpMethod method, const std::string & path = "/", const std::string & body = "",
    const std::unordered_map<std::string, std::string> & headers = {});

  // Signal emitted when HTTP response is received (behaviors connect to this)
  smacc2::SmaccSignal<void(const TResponse &)> onResponseReceived_;

private:
  static constexpr int HTTP_VERSION = 11;

  CpHttpConnectionManager * connectionManager_;
  CpHttpSessionManager * sessionManager_;

  std::function<void(TResponse)> responseHandler_;
};

}  // namespace cl_http
