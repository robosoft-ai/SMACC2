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
#include <boost/statechart/event.hpp>
#include <functional>
#include <smacc2/common.hpp>
#include <smacc2/smacc_event_generator.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <typeinfo>

namespace smacc2
{
namespace event_generators
{
template <typename TSource, typename TState>
struct EvTrue : sc::event<EvTrue<TSource, TState>>
{
};

template <typename TSource, typename TState>
struct EvFalse : sc::event<EvFalse<TSource, TState>>
{
};

enum class ConditionalGeneratorMode
{
  ONE_SHOT,
  ON_UPDATE
};

//-----------------------------------------------------------------------
class EgConditionalGenerator : public smacc2::SmaccEventGenerator, public ISmaccUpdatable
{
public:
  EgConditionalGenerator(
    ConditionalGeneratorMode mode, std::function<bool()> updatePredicate = nullptr);
  virtual ~EgConditionalGenerator();

  void onEntry() override;

  template <typename TState, typename TSource>
  void onStateAllocation()
  {
    this->postEventTrue = [this]() { this->postEvent<EvTrue<TSource, TState>>(); };
    this->postEventFalse = [this]() { this->postEvent<EvFalse<TSource, TState>>(); };
  }

  virtual void update() override;
  ConditionalGeneratorMode mode_;

  void setPredicateFunction(std::function<bool()> updatePredicate);

private:
  void checkPredicateAndPost();
  std::function<void()> postEventTrue;
  std::function<void()> postEventFalse;
  std::function<bool()> updatePredicate_;
};
}  // namespace event_generators
}  // namespace smacc2
