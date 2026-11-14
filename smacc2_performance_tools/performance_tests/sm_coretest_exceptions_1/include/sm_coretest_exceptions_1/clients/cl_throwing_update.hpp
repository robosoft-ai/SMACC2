// Copyright 2025 Robosoft Inc.
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

#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>

#include <sm_coretest_exceptions_1/throw_site.hpp>

namespace sm_coretest_exceptions_1
{
// An updatable client. With the site client_update, every tenth update() throws.
class ClThrowingUpdate : public smacc2::ISmaccClient, public smacc2::ISmaccUpdatable
{
public:
  void update() override
  {
    if (++updates_ % 10 == 0) throwIfSite(getLogger(), "client_update");
  }

private:
  int updates_ = 0;
};
}  // namespace sm_coretest_exceptions_1
