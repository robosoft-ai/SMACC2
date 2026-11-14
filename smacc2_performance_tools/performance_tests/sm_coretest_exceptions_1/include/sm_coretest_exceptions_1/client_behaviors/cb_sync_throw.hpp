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

#include <sm_coretest_exceptions_1/throw_site.hpp>

namespace sm_coretest_exceptions_1
{
// A synchronous behavior that throws from onEntry (site sync_entry) or onExit (site sync_exit).
class CbSyncThrow : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override { throwIfSite(getLogger(), "sync_entry"); }
  void onExit() override { throwIfSite(getLogger(), "sync_exit"); }
};
}  // namespace sm_coretest_exceptions_1
