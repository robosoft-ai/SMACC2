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

#include <eg_random_generator/eg_random_generator.hpp>

namespace smacc2
{
namespace state_reactors
{
EgRandomGenerator::EgRandomGenerator(
  RandomGenerateReactorMode mode, double evAMin, double evAMax, double evBMin, double evBMax,
  double evCMin, double evCMax)
: mode_(mode),
  evAMin_(evAMin),
  evAMax_(evAMax),
  evBMin_(evBMin),
  evBMax_(evBMax),
  evCMin_(evCMin),
  evCMax_(evCMax)
{
  auto values = {evAMin, evAMax, evBMin, evBMax, evCMin, evCMax};

  this->minValue = std::numeric_limits<double>::max();
  this->maxValue = std::numeric_limits<double>::min();
  for (auto & v : values)
  {
    if (v < minValue)
    {
      minValue = v;
    }

    if (v > maxValue)
    {
      maxValue = v;
    }
  }
}

void EgRandomGenerator::postRandomEvents()
{
  int range = this->maxValue - this->minValue;
  int result = (rand() % range) + minValue;

  if (result >= evAMin_ && result <= evAMax_)
  {
    this->postEventA();
  }

  if (result >= evBMin_ && result <= evBMax_)
  {
    this->postEventB();
  }

  if (result >= evCMin_ && result <= evCMax_)
  {
    this->postEventC();
  }
}

void EgRandomGenerator::onEntry()
{
  if (mode_ == RandomGenerateReactorMode::ONE_SHOT)
  {
    this->postRandomEvents();
  }
}

void EgRandomGenerator::update()
{
  if (mode_ == RandomGenerateReactorMode::ON_UPDATE)
  {
    this->postRandomEvents();
  }
}
}  // namespace state_reactors
}  // namespace smacc2
