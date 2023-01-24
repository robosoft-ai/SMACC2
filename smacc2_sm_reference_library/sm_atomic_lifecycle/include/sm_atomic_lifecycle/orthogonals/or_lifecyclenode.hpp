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
#include <lifecyclenode_client/lifecyclenode_client.hpp>

using namespace std::chrono_literals;

namespace sm_atomic_lifecycle
{
using namespace std::chrono_literals;
class OrLifecycleNode : public smacc2::Orthogonal<OrLifecycleNode>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_lifecyclenode::ClLifecycleNode>("lifecycle_example_node");
  }
};
}  // namespace sm_atomic_lifecycle
