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

#pragma once

#include <http_client/cl_http_client.hpp>
#include <http_client/client_behaviors/cb_http_request.hpp>
#include <smacc2/smacc.hpp>

namespace cl_http
{
class CbHttpGetRequest : public CbHttpRequestBase
{
public:
  CbHttpGetRequest() : CbHttpRequestBase(CpHttpRequestExecutor::HttpMethod::GET) {}
};
}  // namespace cl_http
