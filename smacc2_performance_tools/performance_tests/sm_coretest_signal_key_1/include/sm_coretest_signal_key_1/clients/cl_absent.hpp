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

namespace sm_coretest_signal_key_1
{
// A client type that no orthogonal of this state machine holds. StA asks for it through the
// state-level requiresClient on every entry, which drives the core's "client not found" error
// path: the lookup must return a null pointer and the log line must carry the state's name.
class ClAbsent : public smacc2::ISmaccClient
{
};
}  // namespace sm_coretest_signal_key_1
