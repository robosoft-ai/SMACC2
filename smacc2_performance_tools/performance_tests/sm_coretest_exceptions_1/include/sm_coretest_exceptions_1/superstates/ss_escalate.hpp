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

namespace sm_coretest_exceptions_1
{
// A container state. It declares the one reaction for an initialization failure that no state
// inside it handled: the failure reaches it addressed to SsEscalate and sends the machine to StSafe.
// Its own onEntry throws with the site container_entry; StInner is then not initialized either.
struct SsEscalate : smacc2::SmaccState<SsEscalate, SmCoretestExceptions1, StInner>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvStInitializationFailure<SsEscalate>, StSafe>> reactions;

  void onEntry()
  {
    throwIfSite(getLogger(), "container_entry");
    RCLCPP_INFO(getLogger(), "[SsEscalate] Entered.");
  }
};
}  // namespace sm_coretest_exceptions_1
