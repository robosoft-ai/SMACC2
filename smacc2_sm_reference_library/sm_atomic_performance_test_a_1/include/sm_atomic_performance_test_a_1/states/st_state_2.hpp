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

#include <chrono>
#include <optional>
#include <smacc2/smacc.hpp>

// STATE DECLARATION
using namespace std::chrono;

namespace sm_atomic_performance_test_a_1
{
static std::optional<system_clock::time_point> start;
static int count = 0;

struct State2 : smacc2::SmaccState<State2, SmAtomicPerformanceTestA1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvStateRequestFinish<State2>, State1>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  void onEntry()
  {
    using namespace std::chrono;

    count++;
    if (!start)
    {
      start = high_resolution_clock::now();
    }
    else
    {
      auto now = high_resolution_clock::now();
      duration<double, std::milli> elapsed = now - *start;

      if (elapsed.count() > 10000)
      {
        std::cout << "Waited " << elapsed.count() << " ms" << std::endl;
        std::cout << "Number of iterations " << count << std::endl;

        ::exit(0);
      }
    }

    this->postEvent<EvStateRequestFinish<State2>>();
  }

  void onExit() {}
};
}  // namespace sm_atomic_performance_test_a_1
