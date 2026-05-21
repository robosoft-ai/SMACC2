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

#include <smacc2/smacc_orthogonal.hpp>
#include "sm_data_sharing_2/clients/cl_data/cl_data.hpp"

namespace sm_data_sharing_2
{
class OrData : public smacc2::Orthogonal<OrData>
{
public:
  void onInitialize() override { this->createClient<cl_data::ClData>(); }
};
}  // namespace sm_data_sharing_2
