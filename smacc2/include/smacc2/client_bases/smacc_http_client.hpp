// Copyright 2021 RobosoftAI Inc.
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
#include <iostream>
#include <memory>
#include <optional>
#include <smacc2/smacc_client.hpp>
#include <string>
#include <thread>
#include <unordered_map>

namespace smacc2 {
namespace client_bases {
class SmaccHttpClient : public smacc2::ISmaccClient {
  class http_session : public std::enable_shared_from_this<http_session> {
    boost::asio::ip::tcp::resolver resolver_;
    boost::beast::tcp_stream stream_;
    boost::beast::flat_buffer buffer_;  // (Must persist between reads)
    boost::beast::http::request<boost::beast::http::empty_body> req_;
    boost::beast::http::response<boost::beast::http::string_body> res_;

   public:
    using TResponse =
        const boost::beast::http::response<boost::beast::http::string_body> &;

    // Objects are constructed with a strand to
    // ensure that handlers do not execute concurrently.
    http_session(boost::asio::io_context &ioc,
                 const std::function<void(TResponse)> response)
        : resolver_(boost::asio::make_strand(ioc)),
          stream_(boost::asio::make_strand(ioc)),
          onResponse{response} {}

    // Start the asynchronous operation
    void run(const std::string &host, const std::string &target,
             const std::string &port,
             const boost::beast::http::verb http_method, const int &version) {
      // Set up an HTTP request
      req_.version(version);
      req_.method(http_method);
      req_.target(target);
      req_.set(boost::beast::http::field::host, host);
      req_.set(boost::beast::http::field::user_agent,
               BOOST_BEAST_VERSION_STRING);

      // Look up the domain name
      resolver_.async_resolve(
          host.c_str(), port.c_str(),
          boost::beast::bind_front_handler(&http_session::on_resolve,
                                           shared_from_this()));
    }

    void on_resolve(boost::beast::error_code ec,
                    boost::asio::ip::tcp::resolver::results_type results) {
      if (ec) return fail(ec, "resolve");

      // Set a timeout on the operation
      stream_.expires_after(std::chrono::seconds(30));

      // Make the connection on the IP address we get from a lookup
      stream_.async_connect(
          results, boost::beast::bind_front_handler(&http_session::on_connect,
                                                    shared_from_this()));
    }

   private:
    void fail(boost::beast::error_code ec, char const *what) {
      std::cerr << what << ": " << ec.message() << "\n";
      res_.result(boost::beast::http::status::bad_request);
      res_.reason() = ec.message();
      onResponse(res_);
    }

    void on_connect(
        boost::beast::error_code ec,
        boost::asio::ip::tcp::resolver::results_type::endpoint_type) {
      if (ec) return fail(ec, "connect");

      // Set a timeout on the operation
      stream_.expires_after(std::chrono::seconds(30));

      // Send the HTTP request to the remote host
      boost::beast::http::async_write(
          stream_, req_,
          boost::beast::bind_front_handler(&http_session::on_write,
                                           shared_from_this()));
    }

    void on_write(boost::beast::error_code ec, std::size_t bytes_transferred) {
      boost::ignore_unused(bytes_transferred);

      if (ec) return fail(ec, "write");

      // Receive the HTTP response
      boost::beast::http::async_read(
          stream_, buffer_, res_,
          boost::beast::bind_front_handler(&http_session::on_read,
                                           shared_from_this()));
    }

    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) {
      boost::ignore_unused(bytes_transferred);

      if (ec) return fail(ec, "read");

      // Gracefully close the socket
      stream_.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both,
                                ec);

      // not_connected happens sometimes so don't bother reporting it.
      if (ec && ec != boost::beast::errc::not_connected)
        return fail(ec, "shutdown");

      // If we get here then the connection is closed gracefully
      onResponse(res_);
    }

    std::function<void(const boost::beast::http::response<
                       boost::beast::http::string_body> &response)>
        onResponse;
  };

 public:
  enum class kHttpRequestMethod {
    GET = static_cast<int>(boost::beast::http::verb::get),
    POST = static_cast<int>(boost::beast::http::verb::post),
  };

  explicit SmaccHttpClient(const std::string &server, const int &timeout = 1500)
      : worker_guard_{boost::asio::make_work_guard(io_context_)},
        initialized_{false},
        server_name_{server},
        timeout_{timeout} {
    if (!server.substr(0, 5).compare("https")) {
      // User explicitly using http
      is_ssl_ = false;
    } else {
      is_ssl_ = true;
    }
  }

  virtual ~SmaccHttpClient() {
    worker_guard_.reset();
    tcp_connection_runner_.join();
  }

  void configure() {
    if (!initialized_) {
      tcp_connection_runner_ = std::thread{[&]() { io_context_.run(); }};
      this->initialized_ = true;
    }
  }

  template <typename T>
  boost::signals2::connection onResponseReceived(
      void (T::*callback)(const std::string &), T *object) {
    return this->getStateMachine()->createSignalConnection(onResponseReceived_,
                                                           callback, object);
  }

  void makeRequest(const kHttpRequestMethod http_method,
                   const std::string &path = "/") {
    std::make_shared<http_session>(io_context_, callbackHandler)
        ->run(server_name_, path, is_ssl_ ? "443" : "80",
              static_cast<boost::beast::http::verb>(http_method), HTTP_VERSION);
  }

 private:
  const int HTTP_VERSION = 11;

  bool initialized_;
  bool is_ssl_;
  std::string server_name_;
  int timeout_;

  std::thread tcp_connection_runner_;
  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<decltype(io_context_)::executor_type>
      worker_guard_;

  std::function<void(http_session::TResponse)> callbackHandler;

  smacc2::SmaccSignal<void(const std::string &)> onResponseReceived_;
};
}  // namespace client_bases
}  // namespace smacc2
