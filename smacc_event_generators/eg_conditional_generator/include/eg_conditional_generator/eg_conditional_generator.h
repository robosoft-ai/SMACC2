#pragma once
#include <smacc/common.h>
#include <smacc/smacc_event_generator.h>
#include <smacc/smacc_updatable.h>
#include <boost/statechart/event.hpp>
#include <functional>
#include <typeinfo>

namespace smacc
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
class EgConditionalGenerator : public smacc::SmaccEventGenerator, public ISmaccUpdatable
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
}  // namespace smacc
