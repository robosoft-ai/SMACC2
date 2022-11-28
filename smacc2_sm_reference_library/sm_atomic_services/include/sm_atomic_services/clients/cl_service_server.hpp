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

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_server_client.hpp>
#include <std_srvs/srv/empty.hpp>

namespace sm_atomic_services
{
    class ClServiceServer : public smacc2::client_bases::SmaccServiceServerClient<std_srvs::srv::Empty>
    {
        public:
          ClServiceServer(const std::string& service_name) : smacc2::client_bases::SmaccServiceServerClient<std_srvs::srv::Empty>(service_name) {}
    };
}
