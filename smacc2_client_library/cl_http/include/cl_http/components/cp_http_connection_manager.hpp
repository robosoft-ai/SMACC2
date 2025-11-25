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

// HTTP Connection Manager Component - Manages io_context and thread lifecycle
// Authors: Claude (Anthropic AI)

#pragma once

#include <boost/asio/any_io_executor.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>
#include <memory>
#include <smacc2/component.hpp>
#include <thread>

namespace cl_http
{

// Manages io_context and worker thread for HTTP operations
// Provides strand for thread-safe session operations
class CpHttpConnectionManager : public smacc2::ISmaccComponent
{
public:
  CpHttpConnectionManager();
  virtual ~CpHttpConnectionManager();

  void onInitialize() override;

  // Get strand for thread-safe session operations
  boost::asio::any_io_executor getStrand();

  // Access to underlying io_context
  boost::asio::io_context & getIoContext();

private:
  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<decltype(io_context_)::executor_type> worker_guard_;
  std::thread io_thread_;
  bool initialized_;
};

}  // namespace cl_http
