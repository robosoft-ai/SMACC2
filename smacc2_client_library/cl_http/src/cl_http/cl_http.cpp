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
 * 	 Authors: Jaycee Lock & Brett Aldrich
 *
 ******************************************************************************************************************/

#include <cl_http/cl_http.hpp>

namespace cl_http
{
ClHttp::ClHttp(const std::string & server_name, const int & /*timeout*/)
: initialized_{false},
  server_url_{server_name},
  connectionManager_{nullptr},
  sessionManager_{nullptr},
  requestExecutor_{nullptr}
{
  // Timeout parameter kept for API compatibility but unused (managed by components)
}

ClHttp::~ClHttp()
{
  // Thread cleanup handled by CpHttpConnectionManager destructor
}

void ClHttp::onInitialize()
{
  // Components created in onComponentInitialization()
  this->initialized_ = true;
}
}  // namespace cl_http
