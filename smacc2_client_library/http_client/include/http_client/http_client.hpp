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
 * 	 Authors: Jaycee Lock
 *
 ******************************************************************************************************************/

#pragma once

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc.hpp>

#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

namespace cl_http {
class ClHttp : public smacc2::ISmaccClient {
  class http_session : public std::enable_shared_from_this<http_session> {
   public:
    using TResponse =
        const boost::beast::http::response<boost::beast::http::string_body> &;

    // Objects are constructed with a strand to
    // ensure that handlers do not execute concurrently.
    http_session(boost::asio::io_context &ioc,
                 const std::function<void(TResponse)> response);

    // Start the asynchronous operation
    void run(const std::string &host, const std::string &target,
             const std::string &port,
             const boost::beast::http::verb http_method, const int &version);

    void on_resolve(boost::beast::error_code ec,
                    boost::asio::ip::tcp::resolver::results_type results);

   private:
    void fail(boost::beast::error_code ec, char const *what);
    void on_connect(
        boost::beast::error_code ec,
        boost::asio::ip::tcp::resolver::results_type::endpoint_type);
    void on_write(boost::beast::error_code ec, std::size_t bytes_transferred);
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);

    std::function<void(const boost::beast::http::response<
                       boost::beast::http::string_body> &response)>
        onResponse;

    boost::asio::ip::tcp::resolver resolver_;
    boost::beast::tcp_stream stream_;
    boost::beast::flat_buffer buffer_;  // (Must persist between reads)
    boost::beast::http::request<boost::beast::http::empty_body> req_;
    boost::beast::http::response<boost::beast::http::string_body> res_;
  };

 public:
  enum class kHttpRequestMethod {
    GET = static_cast<int>(boost::beast::http::verb::get),
    POST = static_cast<int>(boost::beast::http::verb::post),
  };

  template <typename T>
  boost::signals2::connection onResponseReceived(
      void (T::*callback)(const std::string &), T *object) {
    return this->getStateMachine()->createSignalConnection(onResponseReceived_,
                                                           callback, object);
  }

  explicit ClHttp(const std::string &server, const int &timeout = 1500);

  virtual ~ClHttp();

  void configure();
  void makeRequest(const kHttpRequestMethod http_method,
                   const std::string &path = "/");

 private:
  const int HTTP_VERSION = 11;

  bool initialized_;
  bool is_ssl_;
  int timeout_;
  std::string server_name_;

  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<decltype(io_context_)::executor_type>
      worker_guard_;
  std::thread tcp_connection_runner_;

  std::function<void(http_session::TResponse)> callbackHandler;

  smacc2::SmaccSignal<void(const std::string &)> onResponseReceived_;
};
}  // namespace cl_http
