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
namespace state_reactors
{
template <typename TSource, typename TState>
struct EventA : sc::event<EventA<TSource, TState>>
{
};

template <typename TSource, typename TState>
struct EventB : sc::event<EventB<TSource, TState>>
{
};

template <typename TSource, typename TState>
struct EventC : sc::event<EventC<TSource, TState>>
{
};

enum class RandomGenerateReactorMode
{
  INPUT_EVENT_TRIGGERED,
  ONE_SHOT,
  ON_UPDATE
};

//-----------------------------------------------------------------------
class EgRandomGenerator : public SmaccEventGenerator, public ISmaccUpdatable
{
public:
  EgRandomGenerator(
    RandomGenerateReactorMode mode, double evAMin = 1, double evAMax = 4, double evBMin = 5,
    double evBMax = 8, double evCMin = 9, double evCMax = 12);

  void onEntry() override;

  template <typename TState, typename TSource>
  void onStateAllocation()
  {
    this->postEventA = [this]() { this->postEvent<EventA<TSource, TState>>(); };
    this->postEventB = [this]() { this->postEvent<EventB<TSource, TState>>(); };
    this->postEventC = [this]() { this->postEvent<EventC<TSource, TState>>(); };
  }

  void postRandomEvents();

  void update() override;

  RandomGenerateReactorMode mode_;

private:
  std::function<void()> postEventA;
  std::function<void()> postEventB;
  std::function<void()> postEventC;

  double evAMin_;
  double evAMax_;
  double evBMin_;
  double evBMax_;
  double evCMin_;
  double evCMax_;

  double minValue;
  double maxValue;
};
}  // namespace state_reactors
}  // namespace smacc2
