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

#include <http_client/components/cp_http_session_manager.hpp>

#include <algorithm>

namespace cl_http
{

CpHttpSessionManager::ServerConfig::ServerConfig(const std::string & server_name)
: server_name_(server_name), ssl_(true)
{
  if (!server_name_.substr(0, 7).compare("http://"))
  {
    ssl_ = false;
    server_name_.erase(0, 7);
  }
  else if (!server_name_.substr(0, 8).compare("https://"))
  {
    server_name_.erase(0, 8);
    ssl_ = true;
  }
}

CpHttpSessionManager::CpHttpSessionManager()
: ssl_context_(boost::asio::ssl::context::tlsv13_client), initialized_(false)
{
  ssl_context_.set_default_verify_paths();
  ssl_context_.set_verify_mode(boost::asio::ssl::verify_peer);
}

CpHttpSessionManager::~CpHttpSessionManager() {}

void CpHttpSessionManager::onInitialize() { initialized_ = true; }

void CpHttpSessionManager::setServerUrl(const std::string & server_url)
{
  server_config_ = std::make_unique<ServerConfig>(server_url);
}

std::shared_ptr<http_session_base> CpHttpSessionManager::createSession(
  boost::asio::any_io_executor executor,
  std::function<void(const http_session_base::TResponse &)> callback)
{
  if (!server_config_)
  {
    throw std::runtime_error("Server URL not configured");
  }

  if (server_config_->isSSL())
  {
    return std::make_shared<ssl_http_session>(executor, ssl_context_, callback);
  }
  else
  {
    return std::make_shared<http_session>(executor, callback);
  }
}

bool CpHttpSessionManager::isSSL() const
{
  return server_config_ ? server_config_->isSSL() : false;
}

std::string CpHttpSessionManager::getServerName() const
{
  return server_config_ ? server_config_->getServerName() : "";
}

std::string CpHttpSessionManager::getPort() const
{
  return server_config_ ? server_config_->getPort() : "";
}

}  // namespace cl_http
