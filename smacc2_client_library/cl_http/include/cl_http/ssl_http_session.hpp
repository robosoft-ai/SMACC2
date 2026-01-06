// Copyright 2025 Robosoft Inc.
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
#include <cl_http/http_session_base.hpp>
#include <iostream>
#include <string>
#include <unordered_map>

namespace cl_http
{
class ssl_http_session : public std::enable_shared_from_this<ssl_http_session>,
                         public http_session_base
{
public:
  // Objects are constructed with a strand to
  // ensure that handlers do not execute concurrently.
  ssl_http_session(
    boost::asio::any_io_executor ioc, boost::asio::ssl::context & ssl_context,
    const std::function<void(const TResponse &)> response);

  virtual ~ssl_http_session() {}

  // Start the asynchronous operation
  void run(
    const std::string & host, const std::string & target,
    const boost::beast::http::verb http_method, const int & version) override;

  std::string getPort() override { return kPort; }

private:
  const std::string kPort = "443";

  void on_resolve(
    boost::beast::error_code ec, boost::asio::ip::tcp::resolver::results_type results) override;
  void fail(boost::beast::error_code ec, const char * what) override;
  void on_connect(
    boost::beast::error_code ec,
    boost::asio::ip::tcp::resolver::results_type::endpoint_type) override;
  void on_handshake(boost::beast::error_code ec) override;
  void on_write(boost::beast::error_code ec, std::size_t bytes_transferred) override;
  void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) override;
  void on_shutdown(boost::beast::error_code ec) override;
  void setBody(const std::string & body) override;
  void setHeaders(const std::unordered_map<std::string, std::string> & headers) override;
  void appendToHeader(const std::string & key, const std::string & val);

  std::function<void(const TResponse &)> onResponse;

  boost::asio::ip::tcp::resolver resolver_;
  // boost::beast::tcp_stream stream_;
  boost::beast::ssl_stream<boost::beast::tcp_stream> stream_;
  boost::beast::flat_buffer buffer_;  // (Must persist between reads)
  boost::beast::http::request<boost::beast::http::string_body> req_;
  TResponse res_;
};
}  // namespace cl_http
