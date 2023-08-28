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

#include <http_client/http_client.hpp>

namespace cl_http {

///////////////////////////
// ClHttp implementation //
// ////////////////////////

ClHttp::ClHttp(const std::string &server, const int &timeout)
    : initialized_{false},
      timeout_{timeout},
      server_name_{server},
      worker_guard_{boost::asio::make_work_guard(io_context_)} {
  // User explicitly using http
  is_ssl_ = server.substr(0, 5).compare("https") ? true : false;
}

ClHttp::~ClHttp() {
  worker_guard_.reset();
  tcp_connection_runner_.join();
}

void ClHttp::configure() {
  if (!initialized_) {
    tcp_connection_runner_ = std::thread{[&]() { io_context_.run(); }};
    this->initialized_ = true;
  }
}

void ClHttp::makeRequest(const kHttpRequestMethod http_method,
                         const std::string &path) {
  std::make_shared<http_session>(io_context_, callbackHandler)
      ->run(server_name_, path, is_ssl_ ? "443" : "80",
            static_cast<boost::beast::http::verb>(http_method), HTTP_VERSION);
}

/////////////////////////////////
// http_session implementation //
/////////////////////////////////

ClHttp::http_session::http_session(
    boost::asio::io_context &ioc, const std::function<void(TResponse)> response)
    : onResponse{response},
      resolver_(boost::asio::make_strand(ioc)),
      stream_(boost::asio::make_strand(ioc)) {}

void ClHttp::http_session::run(const std::string &host,
                               const std::string &target,
                               const std::string &port,
                               const boost::beast::http::verb http_method,
                               const int &version) {
  // Set up an HTTP request
  req_.version(version);
  req_.method(http_method);
  req_.target(target);
  req_.set(boost::beast::http::field::host, host);
  req_.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);

  // Look up the domain name
  resolver_.async_resolve(host.c_str(), port.c_str(),
                          boost::beast::bind_front_handler(
                              &http_session::on_resolve, shared_from_this()));
}

void ClHttp::http_session::on_resolve(
    boost::beast::error_code ec,
    boost::asio::ip::tcp::resolver::results_type results) {
  if (ec) return fail(ec, "resolve");

  // Set a timeout on the operation
  stream_.expires_after(std::chrono::seconds(30));

  // Make the connection on the IP address we get from a lookup
  stream_.async_connect(
      results, boost::beast::bind_front_handler(&http_session::on_connect,
                                                shared_from_this()));
}
void ClHttp::http_session::fail(boost::beast::error_code ec, char const *what) {
  std::cerr << what << ": " << ec.message() << "\n";
  res_.result(boost::beast::http::status::bad_request);
  res_.reason() = ec.message();
  onResponse(res_);
}

void ClHttp::http_session::on_connect(
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

void ClHttp::http_session::on_write(boost::beast::error_code ec,
                                    std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);

  if (ec) return fail(ec, "write");

  // Receive the HTTP response
  boost::beast::http::async_read(
      stream_, buffer_, res_,
      boost::beast::bind_front_handler(&http_session::on_read,
                                       shared_from_this()));
}

void ClHttp::http_session::on_read(boost::beast::error_code ec,
                                   std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);

  if (ec) return fail(ec, "read");

  // Gracefully close the socket
  stream_.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);

  // not_connected happens sometimes so don't bother reporting it.
  if (ec && ec != boost::beast::errc::not_connected)
    return fail(ec, "shutdown");

  // If we get here then the connection is closed gracefully
  onResponse(res_);
}
}  // namespace cl_http
