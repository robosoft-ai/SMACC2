#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_reactor.h>
#include <boost/statechart/event.hpp>
#include <map>
#include <typeinfo>

namespace smacc
{
namespace state_reactors
{
template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct EvCountdownEnd : sc::event<EvCountdownEnd<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class SrEventCountdown : public StateReactor
{
private:
  std::map<const std::type_info *, bool> triggeredEvents;
  int eventCount_;

public:
  SrEventCountdown(int eventCount);

  virtual void onInitialized() override;

  virtual void onEventNotified(const std::type_info * eventType) override;

  virtual bool triggers() override;
};
}  // namespace state_reactors
}  // namespace smacc
