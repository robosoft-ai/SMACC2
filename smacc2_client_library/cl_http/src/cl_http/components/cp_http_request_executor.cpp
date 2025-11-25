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

/*****************************************************************************************************************
 *
 * 	 Authors: Claude (Anthropic AI)
 *
 ******************************************************************************************************************/

#include <cl_http/components/cp_http_connection_manager.hpp>
#include <cl_http/components/cp_http_request_executor.hpp>
#include <cl_http/components/cp_http_session_manager.hpp>

namespace cl_http
{

CpHttpRequestExecutor::CpHttpRequestExecutor()
: connectionManager_(nullptr), sessionManager_(nullptr)
{
  responseHandler_ = [this](const TResponse & res) { onResponseReceived_(res); };
}

CpHttpRequestExecutor::~CpHttpRequestExecutor() {}

void CpHttpRequestExecutor::onInitialize()
{
  // Dependencies set via setDependencies() before onInitialize() is called
}

void CpHttpRequestExecutor::executeRequest(
  const HttpMethod method, const std::string & path, const std::string & body,
  const std::unordered_map<std::string, std::string> & headers)
{
  auto path_used = path;
  if (path.empty() || path[0] != '/')
  {
    path_used = "/" + path;
  }

  RCLCPP_INFO(
    this->getLogger(), "Executing request: SSL=%d Server=%s Path=%s Port=%s",
    sessionManager_->isSSL(), sessionManager_->getServerName().c_str(), path_used.c_str(),
    sessionManager_->getPort().c_str());

  auto executor = connectionManager_->getStrand();
  auto session = sessionManager_->createSession(executor, responseHandler_);

  session->setBody(body);
  session->setHeaders(headers);
  session->run(
    sessionManager_->getServerName(), path_used, static_cast<boost::beast::http::verb>(method),
    HTTP_VERSION);
}

}  // namespace cl_http
