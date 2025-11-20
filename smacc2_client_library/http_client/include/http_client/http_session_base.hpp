// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
*

Author: Jacobus Lock

******************************************************************************************************************/

#pragma once

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/version.hpp>
#include <string>

namespace cl_http
{
class http_session_base
{
public:
  virtual ~http_session_base() {}

  using TResponse = boost::beast::http::response<boost::beast::http::string_body>;

  // Start the asynchronous operation
  virtual void run(
    const std::string & host, const std::string & target,
    const boost::beast::http::verb http_method, const int & version) = 0;

  virtual std::string getPort() = 0;
  virtual void setBody(const std::string & body) = 0;
  virtual void setHeaders(const std::unordered_map<std::string, std::string> & headers) = 0;

protected:
  virtual void on_resolve(
    boost::beast::error_code ec, boost::asio::ip::tcp::resolver::results_type results) = 0;
  virtual void fail(boost::beast::error_code ec, const char * what) = 0;
  virtual void on_connect(
    boost::beast::error_code ec, boost::asio::ip::tcp::resolver::results_type::endpoint_type) = 0;
  virtual void on_write(boost::beast::error_code ec, std::size_t bytes_transferred) = 0;
  virtual void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) = 0;

  // Optional, needed for SSL connections
  virtual void on_handshake(boost::beast::error_code /*ec*/) {}
  virtual void on_shutdown(boost::beast::error_code /*ec*/) {}
};
}  // namespace cl_http
