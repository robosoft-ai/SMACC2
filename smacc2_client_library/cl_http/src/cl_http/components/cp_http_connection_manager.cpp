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

namespace cl_http
{

CpHttpConnectionManager::CpHttpConnectionManager()
: worker_guard_(boost::asio::make_work_guard(io_context_.get_executor())), initialized_(false)
{
}

CpHttpConnectionManager::~CpHttpConnectionManager()
{
  if (initialized_)
  {
    worker_guard_.reset();
    if (io_thread_.joinable())
    {
      io_thread_.join();
    }
  }
}

void CpHttpConnectionManager::onInitialize()
{
  if (!initialized_)
  {
    io_thread_ = std::thread([this]() { io_context_.run(); });
    initialized_ = true;
  }
}

boost::asio::any_io_executor CpHttpConnectionManager::getStrand()
{
  return boost::asio::make_strand(io_context_);
}

boost::asio::io_context & CpHttpConnectionManager::getIoContext() { return io_context_; }

}  // namespace cl_http
