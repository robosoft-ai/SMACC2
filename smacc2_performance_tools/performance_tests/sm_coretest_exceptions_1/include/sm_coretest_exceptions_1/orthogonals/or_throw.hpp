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

#include <sm_coretest_exceptions_1/clients/cl_throwing_update.hpp>

namespace sm_coretest_exceptions_1
{
class OrThrow : public smacc2::Orthogonal<OrThrow>
{
public:
  void onInitialize() override { this->createClient<ClThrowingUpdate>(); }
};
}  // namespace sm_coretest_exceptions_1
