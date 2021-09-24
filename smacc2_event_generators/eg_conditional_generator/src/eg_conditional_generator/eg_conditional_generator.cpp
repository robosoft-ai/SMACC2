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

#include <eg_conditional_generator/eg_conditional_generator.hpp>

namespace smacc2
{
namespace event_generators
{
EgConditionalGenerator::EgConditionalGenerator(
  ConditionalGeneratorMode mode, std::function<bool()> updatePredicate)
: mode_(mode), updatePredicate_(updatePredicate)
{
}

EgConditionalGenerator::~EgConditionalGenerator() {}

void EgConditionalGenerator::checkPredicateAndPost()
{
  if (this->updatePredicate_())
  {
    this->postEventTrue();
  }
  else
  {
    this->postEventFalse();
  }
}

void EgConditionalGenerator::onEntry()
{
  if (mode_ == ConditionalGeneratorMode::ONE_SHOT)
  {
    this->checkPredicateAndPost();
  }
}

void EgConditionalGenerator::update()
{
  if (mode_ == ConditionalGeneratorMode::ON_UPDATE)
  {
    this->checkPredicateAndPost();
  }
}

void EgConditionalGenerator::setPredicateFunction(std::function<bool()> updatePredicate)
{
  updatePredicate_ = updatePredicate;
}

}  // namespace event_generators
}  // namespace smacc2
