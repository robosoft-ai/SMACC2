// Copyright 2026 RobosoftAI Inc.
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

namespace sm_coretest_idle_load_1
{
// The only state. Its one transition is on an event nothing posts, so it is never left.
struct StIdle : smacc2::SmaccState<StIdle, SmCoretestIdleLoad1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvCbFailure<CbMessageCounter, OrLoad>, StIdle>> reactions;

  static void staticConfigure() { configure_orthogonal<OrLoad, CbMessageCounter>(); }
};
}  // namespace sm_coretest_idle_load_1
