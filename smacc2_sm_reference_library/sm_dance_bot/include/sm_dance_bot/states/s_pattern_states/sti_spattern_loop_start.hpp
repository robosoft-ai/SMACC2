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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

namespace sm_dance_bot
{
namespace s_pattern_states
{
template <class TDerived, typename TContext>
struct B : smacc2::SmaccState<TDerived, TContext>
{
public:
  using SmaccState<TDerived, TContext>::SmaccState;
  //typedef typename TDerived::reactions reactions;
};

// STATE DECLARATION
struct StiSPatternLoopStart : public B<StiSPatternLoopStart, SS>
{
  using B::B;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopContinue<StiSPatternLoopStart>, StiSPatternRotate1, CONTINUELOOP>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  bool loopCondition()
  {
    auto & superstate = this->context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry() { checkWhileLoopConditionAndThrowEvent(&StiSPatternLoopStart::loopCondition); }
};

}  // namespace s_pattern_states
}  // namespace sm_dance_bot
