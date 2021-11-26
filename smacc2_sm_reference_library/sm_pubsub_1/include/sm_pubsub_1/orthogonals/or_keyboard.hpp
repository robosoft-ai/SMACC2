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

#pragma once

#include <keyboard_client/cl_keyboard.hpp>
#include <smacc2/smacc_orthogonal.hpp>
namespace sm_pubsub_1
{
class OrKeyboard : public smacc2::Orthogonal<OrKeyboard>
{
public:
  void onInitialize() override { auto clKeyboard = this->createClient<cl_keyboard::ClKeyboard>(); }
};
}  // namespace sm_pubsub_1
