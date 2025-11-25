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

// HTTP Session Manager Component - Manages SSL context and session creation
// Authors: Claude (Anthropic AI)

#pragma once

#include <boost/asio/ssl/context.hpp>
#include <http_client/http_session.hpp>
#include <http_client/http_session_base.hpp>
#include <http_client/ssl_http_session.hpp>
#include <memory>
#include <smacc2/component.hpp>
#include <string>

namespace cl_http
{

// Manages SSL context and creates HTTP/HTTPS sessions based on URL
// Parses server URL to determine protocol and port
class CpHttpSessionManager : public smacc2::ISmaccComponent
{
public:
  CpHttpSessionManager();
  virtual ~CpHttpSessionManager();

  void onInitialize() override;

  // Configure server URL (parses protocol, host, port)
  void setServerUrl(const std::string & server_url);

  // Create HTTP or HTTPS session based on URL configuration
  std::shared_ptr<http_session_base> createSession(
    boost::asio::any_io_executor executor,
    std::function<void(const http_session_base::TResponse &)> callback);

  // Check if HTTPS is required
  bool isSSL() const;

  // Get server name without protocol
  std::string getServerName() const;

  // Get port number (443 for HTTPS, 80 for HTTP)
  std::string getPort() const;

private:
  // Internal class to parse and store server configuration
  class ServerConfig
  {
  public:
    explicit ServerConfig(const std::string & server_name);

    bool isSSL() const { return ssl_; }
    std::string getPort() const { return ssl_ ? "443" : "80"; }
    std::string getServerName() const { return server_name_; }

  private:
    std::string server_name_;
    bool ssl_;
  };

  std::unique_ptr<ServerConfig> server_config_;
  boost::asio::ssl::context ssl_context_;
  bool initialized_;
};

}  // namespace cl_http
